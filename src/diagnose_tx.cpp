// Transmit diagnostic for the non-acknowledgement problem.
//
// The motor answers its own traffic but never acknowledges our frames, and thousands
// of transmits were rejected. The TWAI error counters can separate the two candidate
// causes without needing a scope:
//
//   - no acknowledgement: each ACK error adds 8 to the transmit error counter, so TEC
//     climbs in steps of 8 and the frame never completes.
//   - physical layer or bit timing: bit, stuff and form errors dominate instead.
//
// This build also uses single shot transmission (message.ss = 1) so a frame that is
// not acknowledged is abandoned rather than retried forever, which is what wedged the
// transmit buffer in earlier attempts.
//
// It never transmits on its own: it boots in LISTEN ONLY and only sends after a 2
// second BOOT press, then reports the counters before and after.

#include <Arduino.h>
#include "driver/twai.h"

#ifndef CAN_BITRATE
#define CAN_BITRATE 1000000
#endif
#if CAN_BITRATE == 1000000
#define DIAG_TIMING TWAI_TIMING_CONFIG_1MBITS
#elif CAN_BITRATE == 500000
#define DIAG_TIMING TWAI_TIMING_CONFIG_500KBITS
#else
#error Unsupported CAN bitrate
#endif

static const gpio_num_t PIN_TX = GPIO_NUM_4;
static const gpio_num_t PIN_RX = GPIO_NUM_5;
static const gpio_num_t BUTTON_GPIO = GPIO_NUM_0;
static const unsigned long LONG_PRESS_MS = 2000;

#ifndef DIAG_TX_QUEUE_LEN
#define DIAG_TX_QUEUE_LEN 0
#endif

// The bring-up request that the motor never acknowledged.
static const uint8_t REQUEST[13] = {
    0x55, 0x0d, 0x04, 0x33, 0xe5, 0xbf, 0x00, 0x00, 0x40, 0x00, 0x01, 0xdd, 0xfe,
};

struct Counters {
    uint32_t tec, rec, bus_errors, arb_lost, tx_failed, rx_missed, state, msgs_to_tx;
};

static Counters read_counters() {
    twai_status_info_t status{};
    Counters counters{};
    if (twai_get_status_info(&status) == ESP_OK) {
        counters.tec = status.tx_error_counter;
        counters.rec = status.rx_error_counter;
        counters.bus_errors = status.bus_error_count;
        counters.arb_lost = status.arb_lost_count;
        counters.tx_failed = status.tx_failed_count;
        counters.rx_missed = status.rx_missed_count;
        counters.state = (uint32_t)status.state;
        counters.msgs_to_tx = status.msgs_to_tx;
    }
    return counters;
}

static void print_counters(const char *label, const Counters &counters) {
    Serial.printf("# %s tec=%u rec=%u bus_errors=%u arb_lost=%u tx_failed=%u "
                  "rx_missed=%u state=%u msgs_to_tx=%u\n",
                  label, counters.tec, counters.rec, counters.bus_errors,
                  counters.arb_lost, counters.tx_failed, counters.rx_missed,
                  counters.state, counters.msgs_to_tx);
}

static bool button_down() { return digitalRead(BUTTON_GPIO) == LOW; }

static bool set_mode(bool transmit) {
    twai_stop();
    twai_driver_uninstall();
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        PIN_TX, PIN_RX, transmit ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY);
    general.tx_queue_len = transmit ? DIAG_TX_QUEUE_LEN : 0;
    general.rx_queue_len = 128;
    twai_timing_config_t timing = DIAG_TIMING();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t result = twai_driver_install(&general, &timing, &filter);
    if (result == ESP_OK) {
        result = twai_start();
    }
    if (result != ESP_OK) {
        Serial.printf("# ERROR mode_switch %s\n", esp_err_to_name(result));
        return false;
    }
    return true;
}

// One frame, single shot, so a missing acknowledgement cannot wedge the buffer.
static esp_err_t send_one(uint32_t identifier, const uint8_t *data, uint8_t length) {
    twai_message_t message{};
    message.identifier = identifier;
    message.data_length_code = length;
    message.ss = 1;
    message.self = 0;
    for (uint8_t index = 0; index < length; ++index) {
        message.data[index] = data[index];
    }
    return twai_transmit(&message, pdMS_TO_TICKS(50));
}

static void run_diagnostic() {
    Serial.println("# DIAG begin");
    Counters before = read_counters();
    print_counters("before", before);

    esp_err_t result = send_one(0x503, REQUEST, 8);
    Serial.printf("# DIAG send1 result=%s\n", esp_err_to_name(result));
    delay(200);
    Counters after_first = read_counters();
    print_counters("after_send1", after_first);
    Serial.printf("# DIAG delta tec=%d rec=%d bus_errors=%d arb_lost=%d tx_failed=%d\n",
                  (int)(after_first.tec - before.tec),
                  (int)(after_first.rec - before.rec),
                  (int)(after_first.bus_errors - before.bus_errors),
                  (int)(after_first.arb_lost - before.arb_lost),
                  (int)(after_first.tx_failed - before.tx_failed));

    for (int index = 0; index < 5; ++index) {
        result = send_one(0x503, REQUEST, 8);
        Serial.printf("# DIAG repeat %d result=%s\n", index, esp_err_to_name(result));
        delay(150);
    }
    Counters after_all = read_counters();
    print_counters("after_repeats", after_all);
    Serial.printf("# DIAG total tec=%d bus_errors=%d arb_lost=%d tx_failed=%d\n",
                  (int)(after_all.tec - before.tec),
                  (int)(after_all.bus_errors - before.bus_errors),
                  (int)(after_all.arb_lost - before.arb_lost),
                  (int)(after_all.tx_failed - before.tx_failed));
    Serial.println("# DIAG tec rising in steps of 8 with bus_errors tracking it means "
                   "no acknowledgement; bit or stuff errors instead point at the "
                   "physical layer or bit timing");
    set_mode(false);
    Serial.println("# DIAG done, back to listen only");
}

void setup() {
    pinMode(PIN_TX, OUTPUT);
    digitalWrite(PIN_TX, HIGH);
    pinMode(BUTTON_GPIO, INPUT_PULLUP);
    Serial.begin(921600);
    delay(1000);
    Serial.println("# START twai_diagnostic");
    Serial.println("# BUTTON hold 2s to run the transmit diagnostic");
    if (!set_mode(false)) return;
    Serial.println("# READY listen_only");
}

void loop() {
    static bool was_down = false;
    static bool fired = false;
    static unsigned long pressed_at = 0;
    bool down = button_down();
    if (down && !was_down) {
        pressed_at = millis();
        fired = false;
    } else if (down && !fired && millis() - pressed_at >= LONG_PRESS_MS) {
        fired = true;
        Serial.println("# DIAG button held, switching to transmit");
        if (set_mode(true)) {
            run_diagnostic();
        }
    }
    was_down = down;
    delay(5);
}
