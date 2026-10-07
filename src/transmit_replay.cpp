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
#include <cstring>
#include "driver/twai.h"
#include "esp_timer.h"

// --- DUML checksums -----------------------------------------------------------
//
// Same algorithm as tools/analyze_protocol.py, so the bytes this firmware emits can
// be checked against the captures with the project's own tools. The header CRC8 uses
// init 0x77 / reflected polynomial 0x8C over the first three bytes; the packet CRC16
// uses init 0x3692 / reflected polynomial 0x8408 over the whole packet except its
// trailing two bytes, stored little endian.
static uint16_t dji_crc(const uint8_t *data, unsigned length, uint16_t initial,
                        uint16_t polynomial) {
    uint16_t value = initial;
    for (unsigned index = 0; index < length; ++index) {
        value ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            value = (value & 1) ? (uint16_t)((value >> 1) ^ polynomial) : (value >> 1);
        }
    }
    return value;
}

static uint8_t crc8(const uint8_t *data, unsigned length) {
    return (uint8_t)dji_crc(data, length, 0x77, 0x8C);
}

static uint16_t crc16(const uint8_t *data, unsigned length) {
    return dji_crc(data, length, 0x3692, 0x8408);
}

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

// Duty cycle for the transmitted traffic. The first S4 attempts crashed and rebooted
// roughly every 0.1-0.9 s, which means the motor saw short bursts of our frames
// separated by silence - and that is exactly when the operator reported seeing the
// status LED go green. With the crash fixed and the sender running continuously the
// LED has stayed red. That suggests the motor may want gaps rather than an
// uninterrupted stream, so this cycles the sender between ON and OFF without changing
// any packet content. Set both to 0 to transmit continuously.
#ifndef REPLAY_ON_MS
#define REPLAY_ON_MS 0
#endif
#ifndef REPLAY_OFF_MS
#define REPLAY_OFF_MS 0
#endif

// Reproduce the RS3 startup handshake before settling into the steady dial stream.
// The motor answers the dial command but does not report a healthy link without this,
// so the handshake is on by default for stage 4.
#ifndef REPLAY_HANDSHAKE
#define REPLAY_HANDSHAKE 0
#endif

// How long the gimbal attitude stream (04:05) keeps repeating alongside the
// handshake. The capture shows it running well past the handshake, but a couple of
// seconds is enough to present the same traffic shape.
#ifndef REPLAY_HANDSHAKE_UNTIL_MS
#define REPLAY_HANDSHAKE_UNTIL_MS 3000
#endif

// One-shot handshake milestones, so each packet goes out exactly once per window.
static bool handshake_sent_startup = false;
static bool handshake_sent_gb53 = false;
static bool handshake_sent_request = false;
static unsigned long last_request = 0;

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

// Transmit queue depth in replay mode. Kept at ZERO on purpose.
//
// This is the finding that matters most in this file: with tx_queue_len > 0,
// twai_transmit calls xQueueSend on the driver's TX queue, and that path panics on
// this setup (uxListRemove <- xTaskRemoveFromEventList <- xQueueGenericSend <-
// twai_transmit). Every crashing variant in this project had a non-zero queue; the
// one configuration that runs indefinitely without crashing (only the 00:01 request)
// had tx_queue_len == 0, in which case twai_transmit never enqueues and simply fails
// if the transmit buffer is occupied. send_packet therefore checks the in-flight
// count first and only transmits when the buffer is free.
#ifndef TX_QUEUE_LEN
#define TX_QUEUE_LEN 0
#endif

// Controller mode used while replaying.
//
// NORMAL waits for an acknowledgement on every frame it sends. On this bench the bus
// holds only the focus motor, which does not acknowledge our frames, so each send
// blocks until the timeout and the transmit queue backs up. That backlog is what
// leads into the twai_transmit panic described above send_packet.
//
// NO_ACK does not wait for an acknowledgement at all: the controller treats a frame
// as sent once it is on the wire. That removes the queue backlog this project keeps
// hitting, at the cost of not being able to tell whether the motor accepted a frame
// (which we never could anyway, since it does not acknowledge us).
#ifndef REPLAY_USE_NO_ACK
#define REPLAY_USE_NO_ACK 0
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
// state.
//
// Recovered from captures/20261007_154729_002678_idle.log. RS3 emits this command on
// TWO routes per period, differing only in the logical destination and the CAN ID:
//
//   0x503  E5->BF   (to the motor)
//   0x201  E5->52
//
// Both carry the same payload. The captured sequence numbers start at 6038 on the
// 0x503 route and 6039 on the 0x201 route, and each route advances its own counter by
// two per packet. The bytes below therefore keep their captured sequence numbers and
// are re-stamped per send with an advancing sequence, recomputing both CRCs.
static const uint8_t M0469_TO_MOTOR[19] = {
    0x55, 0x13, 0x04, 0x03, 0xe5, 0xbf, 0x96, 0x17, 0x00, 0x04, 0x69,
    0x0f, 0x04, 0x08, 0x19, 0x00, 0x00, 0xe5, 0x56,
};
static const uint8_t M0469_TO_PEER[19] = {
    0x55, 0x13, 0x04, 0x03, 0xe5, 0x52, 0x97, 0x17, 0x00, 0x04, 0x69,
    0x0f, 0x04, 0x08, 0x19, 0x00, 0x00, 0xab, 0x5b,
};
// Both are 19 bytes in three CAN frames: 8 + 8 + 3.
static const uint8_t M0469_FRAMES[3] = {8, 8, 3};

// Per-route sequence counters, seeded from the captured values and advancing by two
// per packet, matching the observed cadence.
static uint16_t seq_to_motor = 6038;
static uint16_t seq_to_peer = 6039;

// Copy a template packet and re-stamp it with a new sequence, recomputing the header
// CRC8 and the packet CRC16. Without this the frames carry sequence 0 forever, and a
// receiver that expects the sequence to advance can treat every frame as a duplicate.
static void stamp_packet(const uint8_t *source, uint16_t sequence, uint8_t *out) {
    memcpy(out, source, 19);
    out[6] = (uint8_t)(sequence & 0xFF);
    out[7] = (uint8_t)(sequence >> 8);
    out[3] = crc8(out, 3);
    uint16_t packet_crc = crc16(out, 17);
    out[17] = (uint8_t)(packet_crc & 0xFF);
    out[18] = (uint8_t)(packet_crc >> 8);
}

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
        replay ? (REPLAY_USE_NO_ACK ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL)
               : TWAI_MODE_LISTEN_ONLY);
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
    // Wait until the controller has nothing in flight before queueing more. With a
    // disabled TX queue (tx_queue_len = 0) twai_transmit cannot enqueue behind a busy
    // buffer - it would just fail - so the only safe pattern is to check first. This
    // also keeps us out of the FreeRTOS queue wait list, which is where the crash
    // described above send_packet happens.
    for (int spin = 0; spin < 50; ++spin) {
        twai_status_info_t status{};
        if (twai_get_status_info(&status) != ESP_OK) {
            return false;
        }
        if (status.msgs_to_tx == 0) {
            break;
        }
        delay(1);
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
        if (twai_transmit(&message, 0) != ESP_OK) {
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

// The dial command as its three real CAN frames, on both routes RS3 uses. Each route
// advances its own sequence counter, so the frames are re-stamped every time.
static bool send_m0469() {
    uint8_t motor[19];
    uint8_t peer[19];
    stamp_packet(M0469_TO_MOTOR, seq_to_motor, motor);
    stamp_packet(M0469_TO_PEER, seq_to_peer, peer);
    bool ok = send_packet(0x503, motor, M0469_FRAMES, 3);
    ok = send_packet(0x201, peer, M0469_FRAMES, 3) && ok;
    seq_to_motor = (uint16_t)(seq_to_motor + 2);
    seq_to_peer = (uint16_t)(seq_to_peer + 2);
    return ok;
}

// --- Handshake, reproduced from the startup capture ---------------------------
//
// The motor does not accept the steady-state dial stream on its own: RB3 first walks
// it through a handshake. From captures/20261007_214348_391693_capture.log, in order:
//
//    t=0.000s  0x106  04>F1  04:05  len 58  gimbal attitude, repeated ~20 ms
//    t=0.068s  0x201  E5>52  04:69  payload 0F 04 00 00 00 00   (startup variant)
//    t=0.098s  the motor begins answering
//    t=0.110s  0x503  E5>BF  00:01  len 43  payload contains "GB53"
//    t=0.228s  0x503  E5>BF  04:69  payload 0F 04 08 19 00 00   (steady variant)
//    t=0.550s  0x503  E5>BF  00:01  len 13  the repeating request
//
// The 04:05 payload carries live attitude data we cannot reproduce; one captured
// sample is replayed verbatim, which is enough to present the same traffic shape.
static const uint8_t H0405_SAMPLE[58] = {
    0x55, 0x3a, 0x04, 0x70, 0x04, 0xf1, 0x3e, 0x00, 0x00, 0x04, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x00, 0xf4, 0xff, 0x00,
    0x01, 0xbd, 0x68, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x80, 0x3f, 0xcf, 0x77, 0xc6, 0x39, 0x76,
    0x30, 0x1c, 0xb8, 0x55, 0xb1, 0xdd, 0xb8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xc3, 0xaa,
};
static const uint8_t H0405_FRAMES[8] = {8, 8, 8, 8, 8, 8, 8, 2};

// The 04:05 payload is RS3's live gimbal attitude: 15 bytes are fixed (header, type,
// flags, and the 1.0 scale factor at payload[24..27]) while 30 bytes change every
// sample, carrying a quaternion plus angular rates. Replaying one captured sample
// therefore presents permanently stale attitude data. ANALYTIC mode animates that
// changing region with a smooth synthetic motion so the stream looks alive.
//
// These values are NOT RS3's real attitude and are NOT a decode of the captured
// fields; they only move plausibly. If the motor needs genuine data this will not
// satisfy it, but the test separates "the motor wants this stream alive" from "the
// motor needs the real RS3".
#ifndef REPLAY_ANALYTIC_ATTITUDE
#define REPLAY_ANALYTIC_ATTITUDE 0
#endif

// Offsets inside the 45 byte payload that hold changing values, measured across 824
// captured samples.
static const uint8_t H0405_VARYING[30] = {
    0, 1, 2, 3, 4, 5, 8, 12, 13, 14, 15, 16, 17, 20, 22, 24, 25, 26, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
};

// The 43 byte reply that carries "GB53" (ascii), six CAN frames.
static const uint8_t H0001_GB53[43] = {
    0x55, 0x2b, 0x04, 0x58, 0xe5, 0xbf, 0x12, 0x00, 0x80, 0x00, 0x01,
    0x00, 0x01, 0x47, 0x42, 0x35, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x09, 0x07,
    0x50, 0x41, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x49, 0x18,
};
static const uint8_t H0001_GB53_FRAMES[6] = {8, 8, 8, 8, 8, 3};

// The startup variant of the dial command, with the two trailing value bytes zero.
static const uint8_t M0469_STARTUP[19] = {
    0x55, 0x13, 0x04, 0x03, 0xe5, 0x52, 0x06, 0x00, 0x00, 0x04, 0x69,
    0x0f, 0x04, 0x00, 0x00, 0x00, 0x00, 0x90, 0x07,
};
static const uint8_t M0469_STARTUP_FRAMES[3] = {8, 8, 3};

// Build the gimbal attitude packet for the current instant. In ANALYTIC mode the
// changing region is animated with a slow synthetic motion; otherwise one captured
// sample is sent verbatim.
static unsigned long attitude_step = 0;

static void build_attitude(uint8_t *out) {
    memcpy(out, H0405_SAMPLE, 58);
    out[6] = (uint8_t)(attitude_step & 0xFF);
    out[7] = (uint8_t)((attitude_step >> 8) & 0xFF);
    ++attitude_step;
#if REPLAY_ANALYTIC_ATTITUDE
    // A slow drift plus small jitter, so consecutive samples differ by a plausible
    // amount rather than jumping.
    float phase = (float)attitude_step * 0.02f;
    int base = (int)(sinf(phase) * 40.0f);
    for (unsigned index = 0; index < sizeof(H0405_VARYING); ++index) {
        uint8_t offset = H0405_VARYING[index];
        int jitter = (int)(sinf(phase * (1.0f + 0.13f * index) + index) * 12.0f);
        out[11 + offset] = (uint8_t)(base + jitter);
    }
    // Keep the scale factor at 1.0 (payload[24..27]) that the capture always shows.
    out[11 + 24] = 0x00; out[11 + 25] = 0x00;
    out[11 + 26] = 0x80; out[11 + 27] = 0x3f;
#endif
    // Re-stamp both checksums because the header sequence and body changed.
    out[3] = crc8(out, 3);
    uint16_t packet_crc = crc16(out, 56);
    out[56] = (uint8_t)(packet_crc & 0xFF);
    out[57] = (uint8_t)(packet_crc >> 8);
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
        // Burst mode: when both phases are configured, transmit only during the ON
        // phase. Received frames keep being logged during the silent half.
        bool allowed_to_send = true;
#if (REPLAY_ON_MS > 0) && (REPLAY_OFF_MS > 0)
        allowed_to_send = ((millis() - started) % (REPLAY_ON_MS + REPLAY_OFF_MS)) < REPLAY_ON_MS;
#endif
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
        if (allowed_to_send && millis() - last_send >= REPLAY_INTERVAL_MS) {
            last_send = millis();
            bool ok = true;
            unsigned long elapsed = millis() - started;
#if REPLAY_HANDSHAKE
            // Handshake phase, timed as in the capture: the gimbal attitude stream
            // first, the startup dial request at about 60 ms, the GB53 reply at about
            // 110 ms while the attitude stream continues, then the steady dial stream
            // from about 230 ms onward.
            static unsigned long last_attitude = 0;
            if (elapsed < REPLAY_HANDSHAKE_UNTIL_MS) {
                if (millis() - last_attitude >= 20) {
                    last_attitude = millis();
                    uint8_t attitude[58];
                    build_attitude(attitude);
                    ok = send_packet(0x106, attitude, H0405_FRAMES, 8) && ok;
                }
            }
            if (elapsed >= 60 && !handshake_sent_startup) {
                handshake_sent_startup = true;
                ok = send_packet(0x201, M0469_STARTUP, M0469_STARTUP_FRAMES, 3) && ok;
                Serial.println("# HANDSHAKE startup_dial_sent");
            }
            if (elapsed >= 110 && !handshake_sent_gb53) {
                handshake_sent_gb53 = true;
                ok = send_packet(0x503, H0001_GB53, H0001_GB53_FRAMES, 6) && ok;
                Serial.println("# HANDSHAKE gb53_reply_sent");
            }
            if (elapsed < 550 && !handshake_sent_request) {
                if (elapsed >= 500) {
                    handshake_sent_request = true;
                    ok = send_m0001_request() && ok;
                    Serial.println("# HANDSHAKE request_sent");
                }
            } else if (elapsed >= 550 && millis() - last_request >= 500) {
                last_request = millis();
                ok = send_m0001_request() && ok;
            }
#else
            ok = send_m0001_request();
#endif
#if REPLAY_INCLUDE_0469
            // The dial command is repeated at a conservative cadence; see the note
            // on REPLAY_0469_INTERVAL_MS for why it is not the captured 10 ms.
            static unsigned long last_dial = 0;
#if REPLAY_HANDSHAKE
            // During the handshake the startup variant already went out; the steady
            // stream starts at about 230 ms, matching the capture.
            bool steady_time = elapsed >= 230;
#else
            bool steady_time = true;
#endif
            if (steady_time && millis() - last_dial >= REPLAY_0469_INTERVAL_MS) {
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
