// ESP32-S3 passive CAN listener for the RS3 <-> DJI FM03 link, plus a safe
// button-selected transmit mode.
//
// Two SN65HVD230 transceivers share GPIO4 (TX) / GPIO5 (RX) on the breadboard and
// only one is plugged in at a time. Wiring contract, all three points required:
//
//   1. Monitor module (the one with no 120 ohm terminator): tie its Rs pin
//      (SN65HVD230 pin 8) HIGH to 3.3 V. That disables its driver in silicon, so
//      this module can never drive the bus even if the firmware is in transmit
//      mode. Do not leave that Rs floating.
//   2. Transceiver module: leave Rs grounded or floating so it can drive.
//   3. Only the plugged module's CANH/CANL reaches the real bus. Never connect
//      both modules to the bus at once, or the tap adds a third terminator
//      (RS3 and FM03 already terminate both ends at 120 ohm each).
//
// The firmware always boots in LISTEN mode. Transmit mode needs a deliberate
// long press of the on-board BOOT button (GPIO0) while already running, so the
// button can never decide the mode at power-up. Holding BOOT during reset still
// enters the ROM downloader; that is separate and unchanged.
//
// The built-in addressable RGB LED shows the mode: slow blue pulse = listen,
// fast red pulse = transmit-enabled.
//
// This firmware contains no CAN transmit call. Transmit mode only makes sending
// possible; no frame is ever emitted by this code.

#include <Arduino.h>
#include <atomic>
#include "driver/twai.h"
#include "esp_timer.h"

// The DevKitC-1 already pulls BOOT up, so this stays off by default. Enable it
// only after measuring the idle level and confirming the button pulls the pin
// LOW: a button wired to 3.3 V instead of GND would otherwise read as
// permanently pressed and could silently switch the node into transmit mode.
// #define BUTTON_USES_INTERNAL_PULLUP 1

// Board revisions moved the addressable LED: ESP32-S3-DevKitC-1 v1.0 uses GPIO48
// while v1.1 uses GPIO38. This core variant (esp32s3/pins_arduino.h) sets
// PIN_NEOPIXEL to 48, so that is the default here. Override for a v1.1 board with
// -DLED_RGB_GPIO=38, or set LED_RGB_GPIO=-1 to disable LED output entirely.
#ifndef LED_RGB_GPIO
#define LED_RGB_GPIO 48
#endif
#define LED_ENABLED (LED_RGB_GPIO >= 0)

static const uint8_t BUTTON_GPIO = 0;
static const unsigned long LONG_PRESS_MS = 2000;
static const unsigned long LED_TICK_MS = 25;

enum CanMode { MODE_LISTEN = 0, MODE_TRANSMIT = 1 };

struct CapturedFrame {
    int64_t timestamp_us;
    uint32_t sequence;
    twai_message_t frame;
};

static QueueHandle_t log_queue;
static std::atomic<uint32_t> received{0};
static std::atomic<uint32_t> logger_dropped{0};
static bool running = false;

// Written by loop() when the mode changes and read by the receive task, so it
// stays atomic rather than a plain enum.
static std::atomic<uint8_t> can_mode{MODE_LISTEN};

// Cleared around twai_driver_uninstall()/install() so the receive task never
// calls twai_receive() against a driver that is not installed.
static std::atomic<bool> driver_installed{false};

static uint32_t mode_switches = 0;
static uint32_t frames_in_mode = 0;

// Timestamp is taken on driver dequeue, not at the physical frame edge.
static void receive_frames(void *) {
    while (true) {
        if (!driver_installed.load()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
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

static twai_timing_config_t timing_config() {
#if CAN_BITRATE == 1000000
    return TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BITRATE == 500000
    return TWAI_TIMING_CONFIG_500KBITS();
#else
#error Unsupported CAN bitrate
#endif
}

// Only ever called from loop(), never concurrently with a status print.
static bool install_driver(twai_mode_t mode) {
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        GPIO_NUM_4, GPIO_NUM_5, mode);
    general.tx_queue_len = (mode == TWAI_MODE_NORMAL) ? 8 : 0;
    general.rx_queue_len = 512;
    twai_timing_config_t timing = timing_config();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t result = twai_driver_install(&general, &timing, &filter);
    if (result == ESP_OK) result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("# ERROR twai_install=%s mode=%u\n", esp_err_to_name(result),
                      (unsigned)mode);
        return false;
    }
    return true;
}

static bool change_mode(CanMode mode) {
    if ((CanMode)can_mode.load() == mode) return true;
    driver_installed.store(false);
    twai_stop();
    twai_driver_uninstall();
    bool ok = install_driver(mode == MODE_TRANSMIT ? TWAI_MODE_NORMAL
                                                  : TWAI_MODE_LISTEN_ONLY);
    if (!ok) {
        // Never keep running without a driver: fall back to listen-only.
        if (mode == MODE_TRANSMIT && install_driver(TWAI_MODE_LISTEN_ONLY)) {
            can_mode.store(MODE_LISTEN);
            driver_installed.store(true);
            Serial.println("# ERROR transmit_enable_failed fell_back=listen_only");
        } else {
            running = false;
            Serial.println("# ERROR driver_unavailable");
        }
        return false;
    }
    can_mode.store(mode);
    driver_installed.store(true);
    ++mode_switches;
    frames_in_mode = 0;
    Serial.printf("# MODE name=%s tx_queue=%u switch=%u\n",
                  mode == MODE_TRANSMIT ? "transmit" : "listen",
                  mode == MODE_TRANSMIT ? 8u : 0u, mode_switches);
    if (mode == MODE_TRANSMIT) {
        Serial.println("# WARNING transmit_enabled no_can_frame_sent_by_firmware");
    }
    return true;
}

static bool button_down() {
    return digitalRead(BUTTON_GPIO) == LOW;
}

static void service_button() {
    static bool was_down = false;
    static bool long_press_fired = false;
    static unsigned long pressed_at = 0;
    bool down = button_down();
    if (down && !was_down) {
        pressed_at = millis();
        long_press_fired = false;
    } else if (down && !long_press_fired && millis() - pressed_at >= LONG_PRESS_MS) {
        long_press_fired = true;
        change_mode((CanMode)can_mode.load() == MODE_TRANSMIT ? MODE_LISTEN
                                                              : MODE_TRANSMIT);
    }
    was_down = down;
}

static void service_led() {
    static unsigned long last = 0;
    static unsigned long changed_at = 0;
    static CanMode shown = MODE_LISTEN;
    unsigned long now = millis();
    if (now - last < LED_TICK_MS) return;
    last = now;
    CanMode mode = (CanMode)can_mode.load();
    if (mode != shown) {
        shown = mode;
        changed_at = now;
    }
#if LED_ENABLED
    if (mode == MODE_TRANSMIT) {
        // Fast red pulse, solid for the first 1.5 s after switching so the change
        // is unmistakable even at a glance.
        bool solid = now - changed_at < 1500;
        bool on = solid || ((now - changed_at) / 120) % 2 == 0;
        neopixelWrite(LED_RGB_GPIO, on ? 48 : 0, 0, 0);
    } else {
        // Slow blue pulse.
        bool on = ((now - changed_at) / 700) % 2 == 0;
        neopixelWrite(LED_RGB_GPIO, 0, 0, on ? 24 : 0);
    }
#endif
}

static void service_serial_commands() {
    static bool requested = false;
    static unsigned long last_status = 0;
    while (Serial.available()) {
        if (Serial.read() == 's') requested = true;
    }
    if (requested || millis() - last_status >= 1000) {
        requested = false;
        last_status = millis();
        twai_status_info_t status{};
        if (twai_get_status_info(&status) != ESP_OK) {
            Serial.println("# ERROR status_unavailable");
            return;
        }
        Serial.printf("# STATUS t_us=%llu bitrate=%u mode=%s rx=%u logger_dropped=%u "
                      "rx_missed=%u rx_overrun=%u bus_errors=%u pending=%u "
                      "logger_pending=%u mode_switches=%u frames_in_mode=%u\n",
                      (unsigned long long)esp_timer_get_time(), CAN_BITRATE,
                      can_mode.load() == MODE_TRANSMIT ? "transmit" : "listen",
                      received.load(), logger_dropped.load(),
                      status.rx_missed_count, status.rx_overrun_count,
                      status.bus_error_count, status.msgs_to_rx,
                      (unsigned)uxQueueMessagesWaiting(log_queue), mode_switches,
                      frames_in_mode);
    }
}

void setup() {
    // Hold TX recessive before the driver exists. This firmware never transmits;
    // transmit mode only makes transmission possible.
    digitalWrite(4, HIGH);
    pinMode(4, OUTPUT);
    pinMode(BUTTON_GPIO, INPUT
#ifdef BUTTON_USES_INTERNAL_PULLUP
            | PULLUP
#endif
    );
    Serial.begin(921600);
    delay(1000);
    bool idle_pressed = button_down();
    Serial.printf("# START listen_only bitrate=%u tx_gpio=4 rx_gpio=5 button_gpio=%u "
                  "led_rgb_gpio=%u mode=listen\n",
                  CAN_BITRATE, BUTTON_GPIO, LED_RGB_GPIO);
    Serial.println("# FRAME t_us,sequence,id_hex,extended,rtr,dlc,data_hex");
    Serial.println("# timestamp=driver_dequeue_us_since_boot; command=s_status");
    Serial.printf("# BUTTON hold_ms=%lu toggles_mode; boot_always=listen_only\n",
                  LONG_PRESS_MS);
    Serial.printf("# BUTTON idle_level=%s%s\n",
                  idle_pressed ? "LOW" : "HIGH",
                  idle_pressed ? " unexpected_check_wiring" : " expected");

    log_queue = xQueueCreate(1024, sizeof(CapturedFrame));
    if (!log_queue) {
        Serial.println("# ERROR log_queue_allocation");
        return;
    }
    if (!install_driver(TWAI_MODE_LISTEN_ONLY)) return;
    can_mode.store(MODE_LISTEN);
    driver_installed.store(true);
    if (xTaskCreatePinnedToCore(receive_frames, "can_rx", 4096, nullptr,
                              10, nullptr, 0) != pdPASS) {
        twai_stop();
        Serial.println("# ERROR receive_task_allocation");
        return;
    }
#if LED_ENABLED
    neopixelWrite(LED_RGB_GPIO, 0, 0, 0);
#endif
    running = true;    Serial.println("# READY listen_only");
}

void loop() {
    if (!running) {
        delay(1000);
        return;
    }
    CapturedFrame item{};
    if (xQueueReceive(log_queue, &item, pdMS_TO_TICKS(2)) == pdTRUE) {
        char line[128];
        int length = snprintf(line, sizeof(line), "%llu,%u,%08X,%u,%u,%u,",
                              (unsigned long long)item.timestamp_us, item.sequence,
                              item.frame.identifier, item.frame.extd, item.frame.rtr,
                              item.frame.data_length_code);
        if (!item.frame.rtr) {
            for (unsigned i = 0; i < min((unsigned)item.frame.data_length_code, 8u); ++i) {
                length += snprintf(line + length, sizeof(line) - length,
                                   "%02X", item.frame.data[i]);
            }
        }
        line[length++] = '\n';
        Serial.write((const uint8_t *)line, length);
        ++frames_in_mode;
    }
    // Mode switching stops and restarts the TWAI driver, so it must never overlap
    // a status print or a frame write. Both happen above, before this point.
    service_button();
    service_led();
    service_serial_commands();
}
