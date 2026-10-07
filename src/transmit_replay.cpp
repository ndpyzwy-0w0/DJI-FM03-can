// M5 replay firmware: sends bring-up traffic to the focus motor, in stages.
//
// WHY A SEPARATE BUILD: the passive listener (src/main.cpp) must keep no transmit
// capability at all, so this lives in its own file and its own PlatformIO
// environment. Choose the stage at build time from the measured evidence in
// reports/20261008_enable_set_plan.md.
//
// SAFETY (read before flashing):
//  * Boots in LISTEN (blue): nothing is transmitted until a 2 second BOOT press.
//  * The bus must hold ONLY the focus motor, powered from its own adapter. The RS3
//    must be off and its data lines out of the bus, or two masters will fight.
//  * A replay window is bounded (REPLAY_SECONDS) and then stops by itself.
//  * Pressing BOOT during a window stops it immediately.
//  * The LED is red for the whole window: red means "this node is driving the bus".
//
// The receiver task keeps logging whatever the motor sends while the replay runs,
// so a single run shows both what we sent and how the motor reacted.

#include <Arduino.h>
#include <atomic>
#include "driver/twai.h"
#include "esp_timer.h"

#ifndef REPLAY_STAGE
#define REPLAY_STAGE 1
#endif

#ifndef REPLAY_SECONDS
#define REPLAY_SECONDS 15
#endif

#ifndef REPLAY_INTERVAL_MS
#define REPLAY_INTERVAL_MS 500
#endif

// Rapid mode: start a replay window automatically the moment the board boots, and
// repeat far faster than the observed 500 ms cadence. This exists because the
// focus motor floods the bus at ~7600 frames/s once it is up (about 84% of 1 Mbps),
// which leaves no room for a 13 byte frame to be transmitted at all. The only
// window in which a controller can get a frame onto the wire is the brief quiet
// period between the motor powering up and its flood starting (measured at roughly
// 100 ms in the RS3 power-on capture). So the sender must already be running and
// hammering before the motor is switched on.
#ifndef REPLAY_AUTO_START
#define REPLAY_AUTO_START 0
#endif

// Stage 4 adds the periodic dial command (value zero) on top of the bring-up
// request, to test whether that stream is what the motor needs to report a healthy
// link.
#ifndef REPLAY_INCLUDE_0469
#define REPLAY_INCLUDE_0469 0
#endif

// How often to repeat the dial command. The captured cadence is about 10 ms, but
// pushing a three frame packet that often, while the bus also carries motor traffic,
// fills the TWAI transmit queue. When the queue is saturated twai_transmit corrupts
// the driver's queue and panics (see the note above send_packet), which turned the
// first S4 attempt into a crash loop. Space the repeats so the controller has time
// to place each packet on the bus.
#ifndef REPLAY_0469_INTERVAL_MS
#define REPLAY_0469_INTERVAL_MS 150
#endif

// Transmit queue depth in replay mode. Kept deliberately shallow: the panic this
// project keeps hitting happens inside twai_transmit when it has to enqueue behind a
// full queue, so a shallower queue means fewer chances to enqueue into a saturated
// one. Override per environment when testing.
#ifndef TX_QUEUE_LEN
#define TX_QUEUE_LEN 1
#endif

#ifndef LED_RGB_GPIO
#define LED_RGB_GPIO 48
#endif

static const uint8_t PIN_TX = 4;
static const uint8_t PIN_RX = 5;
static const uint8_t BUTTON_GPIO = 0;
static const unsigned long LONG_PRESS_MS = 2000;

#if CAN_BITRATE == 1000000
#define REPLAY_TIMING TWAI_TIMING_CONFIG_1MBITS
#elif CAN_BITRATE == 500000
#define REPLAY_TIMING TWAI_TIMING_CONFIG_500KBITS
#else
#error Unsupported CAN bitrate
#endif

// --- Messages recovered from the captures -------------------------------------
//
// S1: the 00:01 bring-up request, exactly as captured on CAN 0x503 / logical
// E5->BF. One 13 byte CAN frame. Header CRC8 0x33 and packet CRC16 0xfedd were both
// verified against the capture, so the bytes are replayed verbatim rather than
// recomputed. The capture shows this request carried sequence number 0.
static const uint8_t M0001_REQUEST[13] = {
    0x55, 0x0d, 0x04, 0x33, 0xe5, 0xbf, 0x00, 0x00, 0x40, 0x00, 0x01, 0xdd, 0xfe,
};

// S4: the dial command, replayed with the dial value at zero (nothing commanded to
// move) so the test can show whether this stream alone changes the motor's reported
// state. Recovered from captures/20261007_154729_002678_idle.log as CAN 0x503,
// logical E5->BF, 19 bytes in three CAN frames; the sequence number was 0x1796
// there and is re-stamped to 0 here to match the bring-up convention, with both
// CRCs recomputed and verified.
static const uint8_t M0469_PACKET[19] = {
    0x55, 0x13, 0x04, 0x03, 0xe5, 0xbf, 0x00, 0x00, 0x00, 0x04, 0x69,
    0x0f, 0x04, 0x08, 0x19, 0x00, 0x00, 0x6b, 0x56,
};
static const uint8_t M0469_FRAMES[3] = {8, 8, 3};

// --- State ---------------------------------------------------------------------

struct CapturedFrame {
    int64_t timestamp_us;
    uint32_t sequence;
    twai_message_t frame;
};

static QueueHandle_t log_queue;
static std::atomic<uint32_t> received{0};
static std::atomic<uint32_t> logger_dropped{0};
static std::atomic<uint8_t> mode{0};   // 0 = listen only, 1 = replaying
static bool button_usable = true;
static uint32_t replay_sends = 0;
static uint32_t replay_windows = 0;

static void receive_task(void *) {
    while (true) {
        CapturedFrame item{};
        if (twai_receive(&item.frame, pdMS_TO_TICKS(20)) == ESP_OK) {
            item.timestamp_us = esp_timer_get_time();
            item.sequence = ++received;
            if (xQueueSend(log_queue, &item, 0) != pdTRUE) {
                ++logger_dropped;
            }
        }
    }
}

static void write_led(uint8_t red, uint8_t green, uint8_t blue) {
    neopixelWrite(LED_RGB_GPIO, red, green, blue);
}

// Reinstalls the driver so the controller mode matches the desired mode.
// Listen Only cannot transmit, Normal can.
static bool set_mode(bool replay) {
    twai_stop();
    twai_driver_uninstall();
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TX, (gpio_num_t)PIN_RX,
        replay ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY);
    general.tx_queue_len = replay ? TX_QUEUE_LEN : 0;
    general.rx_queue_len = 128;
    twai_timing_config_t timing = REPLAY_TIMING();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t result = twai_driver_install(&general, &timing, &filter);
    if (result == ESP_OK) result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("# ERROR mode_switch=%s want_replay=%u\n",
                      esp_err_to_name(result), (unsigned)replay);
        return false;
    }
    mode.store(replay ? 1 : 0);
    return true;
}

// NOTE ON AN IDF DEFECT: calling twai_transmit() again immediately after it fails
// with a full transmit queue has been observed to corrupt the TWAI queue's internal
// list and panic the board (backtrace: twai_transmit -> xQueueGenericSend ->
// xTaskRemoveFromEventList -> uxListRemove). Both failures seen in this project had
// that shape. So a failed transmit is never retried immediately: this counter makes
// the caller back off, and nothing is sent while it is non-zero.
static uint32_t tx_backoff_ticks = 0;

static bool send_packet(uint32_t identifier, const uint8_t *packet,
                        const uint8_t *frame_lengths, uint8_t frame_count) {
    if (tx_backoff_ticks) {
        return false;
    }
    const uint8_t *cursor = packet;
    for (uint8_t index = 0; index < frame_count; ++index) {
        twai_message_t message{};
        message.identifier = identifier;
        message.extd = 0;
        message.rtr = 0;
        message.self = 0;
        message.data_length_code = frame_lengths[index];
        for (uint8_t byte = 0; byte < frame_lengths[index]; ++byte) {
            message.data[byte] = cursor[byte];
        }
        if (twai_transmit(&message, pdMS_TO_TICKS(20)) != ESP_OK) {
            tx_backoff_ticks = 20;   // ~200 ms at the 10 ms polling step
            return false;
        }
        cursor += frame_lengths[index];
    }
    return true;
}

// The 13 byte request occupies two CAN frames, not one: frame 1 is the first eight
// bytes, frame 2 the remaining five. Putting all 13 bytes in one frame overruns
// twai_message_t.data, which is only eight bytes.
static bool send_m0001_request() {
    static const uint8_t lengths[2] = {8, 5};
    return send_packet(0x503, M0001_REQUEST, lengths, 2);
}

// The dial command as its three real CAN frames.
static bool send_m0469() {
    return send_packet(0x503, M0469_PACKET, M0469_FRAMES, 3);
}

static bool button_down() {
    return digitalRead(BUTTON_GPIO) == LOW;
}

// Set when a long press starts a window: the finger is still on the button at that
// moment, so the window must ignore the button until it has been released once,
// otherwise the press that started the window would immediately stop it.
static bool ignore_button_until_release = false;

static void service_button() {
    static bool was_down = false;
    static bool long_press_fired = false;
    static unsigned long pressed_at = 0;
    bool down = button_down();

    if (!down) {
        ignore_button_until_release = false;
    }
    if (!button_usable) {
        was_down = down;
        return;
    }
    if (down && !was_down) {
        pressed_at = millis();
        long_press_fired = false;
    } else if (down && !long_press_fired && millis() - pressed_at >= LONG_PRESS_MS) {
        long_press_fired = true;
        if (mode.load() == 1) {
            // Handled inside the window loop; nothing to do here.
        } else {
            Serial.printf("# REPLAY stage=%u starting seconds=%u interval_ms=%u\n",
                          (unsigned)REPLAY_STAGE, (unsigned)REPLAY_SECONDS,
                          (unsigned)REPLAY_INTERVAL_MS);
            if (set_mode(true)) {
                ++replay_windows;
                replay_sends = 0;
                ignore_button_until_release = true;
                write_led(48, 0, 0);
            }
        }
    }
    was_down = down;
}

// Runs one bounded replay window; returns when finished or stopped by a fresh press.
static void run_replay_window() {
    unsigned long started = millis();
    unsigned long last_send = 0;
    unsigned long released_at = 0;
    unsigned long rejected = 0;
    bool reported_rejection = false;
    while (mode.load() == 1 && millis() - started < (unsigned long)REPLAY_SECONDS * 1000UL) {
        if (tx_backoff_ticks) {
            --tx_backoff_ticks;
        }
        bool down = button_down();
        if (ignore_button_until_release) {
            if (!down) {
                ignore_button_until_release = false;
                released_at = millis();
            }
        } else if (down && released_at && millis() - released_at > 300) {
            // A fresh press after the starting press was released: stop.
            Serial.println("# REPLAY stopped_by_button");
            break;
        }
        if (millis() - last_send >= REPLAY_INTERVAL_MS) {
            last_send = millis();
            bool ok = send_m0001_request();
#if REPLAY_INCLUDE_0469
            // The dial command is repeated at a conservative cadence; see the note
            // on REPLAY_0469_INTERVAL_MS for why it is not the captured 10 ms.
            static unsigned long last_dial = 0;
            if (millis() - last_dial >= REPLAY_0469_INTERVAL_MS) {
                last_dial = millis();
                ok = send_m0469() && ok;
            }
#endif
            if (ok) {
                ++replay_sends;
                reported_rejection = false;
            } else {
                // The queue stays full while a frame cannot get onto the bus, so
                // report the condition once per burst instead of every attempt.
                ++rejected;
                if (!reported_rejection) {
                    reported_rejection = true;
                    Serial.println("# WARN transmit_rejected queue_full_bus_monopolised");
                }
            }
        }
        // Keep draining received frames while replaying.
        CapturedFrame item{};
        if (xQueueReceive(log_queue, &item, pdMS_TO_TICKS(5)) == pdTRUE) {
            char line[128];
            int length = snprintf(line, sizeof(line), "%llu,%u,%08X,%u,%u,%u,",
                                  (unsigned long long)item.timestamp_us, item.sequence,
                                  item.frame.identifier, item.frame.extd, item.frame.rtr,
                                  item.frame.data_length_code);
            for (unsigned i = 0; i < min((unsigned)item.frame.data_length_code, 8u); ++i) {
                length += snprintf(line + length, sizeof(line) - length, "%02X",
                                   item.frame.data[i]);
            }
            line[length++] = '\n';
            Serial.write((const uint8_t *)line, length);
        }
    }
    twai_status_info_t status{};
    twai_get_status_info(&status);
    Serial.printf("# REPLAY window_done sent=%u rejected=%u rx=%u bus_errors=%u "
                  "tx_errors=%u rx_missed=%u logger_dropped=%u\n",
                  replay_sends, rejected, received.load(), status.bus_error_count,
                  status.tx_error_counter, status.rx_missed_count,
                  logger_dropped.load());
    set_mode(false);
    write_led(0, 0, 24);
}

static void service_status() {
    static unsigned long last = 0;
    if (millis() - last < 1000) return;
    last = millis();
    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) return;
    Serial.printf("# STATUS t_us=%llu mode=%s stage=%u rx=%u sent=%u windows=%u "
                  "logger_dropped=%u rx_missed=%u bus_errors=%u pending=%u\n",
                  (unsigned long long)esp_timer_get_time(),
                  mode.load() == 1 ? "replay" : "listen", (unsigned)REPLAY_STAGE,
                  received.load(), replay_sends, replay_windows,
                  logger_dropped.load(), status.rx_missed_count,
                  status.bus_error_count, status.msgs_to_rx);
}

void setup() {
    digitalWrite(PIN_TX, HIGH);
    pinMode(PIN_TX, OUTPUT);
    pinMode(BUTTON_GPIO, INPUT_PULLUP);
    Serial.begin(921600);
    delay(1000);
    button_usable = !button_down();
    Serial.printf("# START replay_firmware stage=%u bitrate=%u seconds=%u "
                  "interval_ms=%u\n",
                  (unsigned)REPLAY_STAGE, CAN_BITRATE, (unsigned)REPLAY_SECONDS,
                  (unsigned)REPLAY_INTERVAL_MS);
    Serial.println("# FRAME t_us,sequence,id_hex,extended,rtr,dlc,data_hex");
    Serial.println("# BUTTON hold 2s to start a replay window; hold again to stop");
    if (!button_usable) {
        Serial.println("# ERROR button_idle_low replay_disabled_staying_listen_only");
    }
    log_queue = xQueueCreate(1024, sizeof(CapturedFrame));
    if (!log_queue) {
        Serial.println("# ERROR log_queue_allocation");
        return;
    }
    if (!set_mode(false)) return;
    if (xTaskCreatePinnedToCore(receive_task, "can_rx", 4096, nullptr, 10,
                               nullptr, 0) != pdPASS) {
        Serial.println("# ERROR receive_task_allocation");
        return;
    }
    write_led(0, 0, 24);   // blue = listen only, cannot transmit
    Serial.println("# READY listen_only");
#if REPLAY_AUTO_START
    // Rapid mode: begin hammering immediately so our frames are already going out
    // when the motor is switched on, before its flood monopolises the bus.
    Serial.println("# AUTO_START beginning replay window now");
    if (set_mode(true)) {
        ++replay_windows;
        replay_sends = 0;
        ignore_button_until_release = !button_down();
        write_led(48, 0, 0);
    }
#endif
}

void loop() {
    if (mode.load() == 1) {
        run_replay_window();
    }
    // Drain and log frames while idle.
    CapturedFrame item{};
    if (xQueueReceive(log_queue, &item, pdMS_TO_TICKS(5)) == pdTRUE) {
        char line[128];
        int length = snprintf(line, sizeof(line), "%llu,%u,%08X,%u,%u,%u,",
                              (unsigned long long)item.timestamp_us, item.sequence,
                              item.frame.identifier, item.frame.extd, item.frame.rtr,
                              item.frame.data_length_code);
        for (unsigned i = 0; i < min((unsigned)item.frame.data_length_code, 8u); ++i) {
            length += snprintf(line + length, sizeof(line) - length, "%02X",
                               item.frame.data[i]);
        }
        line[length++] = '\n';
        Serial.write((const uint8_t *)line, length);
    }
    service_button();
    service_status();
}
