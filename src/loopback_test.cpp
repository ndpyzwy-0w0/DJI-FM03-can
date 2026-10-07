// Loopback self-test for the transmit path (M4).
//
// Purpose: prove the ESP32 can build and put correctly formed CAN frames on the
// wire *before* anything is connected to the focus motor.
//
// How the loop closes: an SN65HVD230 feeds TX back to RX internally, and the
// controller's RX reflects both what the transceiver is driving and what the bus
// carries. So a controller in NO_ACK mode with self reception enabled receives its
// own frames. No jumper between the transceiver's D and R pins is required.
//
// This is a DIFFERENT firmware from src/main.cpp (the listener). It is selected by
// the `loopback_test` PlatformIO environment and must never replace the listener
// build used for passive capture.
//
// Safety: run this with the transceiver attached to the ESP32 and NOTHING else on
// the bus - no focus motor, no RS3, no other node. No frame may be sent while the
// RS3 is talking to the motor.
//
// The third check is the important one for M3: it transmits the exact 00:01
// bring-up request bytes recovered from the capture and verifies the controller
// receives them unchanged, which includes their two CRC bytes.

#include <Arduino.h>
#include <atomic>
#include "driver/twai.h"
#include "esp_timer.h"

#if CAN_BITRATE == 1000000
#define LOOPBACK_TIMING TWAI_TIMING_CONFIG_1MBITS
#elif CAN_BITRATE == 500000
#define LOOPBACK_TIMING TWAI_TIMING_CONFIG_500KBITS
#else
#error Unsupported CAN bitrate
#endif

static const uint8_t PIN_TX = 4;
static const uint8_t PIN_RX = 5;

// The 00:01 bring-up request recovered from the capture, split into the two CAN
// frames it occupies (13 bytes total: 8 + 5). Bytes were verified against the
// capture's header CRC8 and packet CRC16.
static const uint8_t BRINGUP_REQUEST[13] = {
    0x55, 0x0d, 0x04, 0x33, 0xe5, 0xbf, 0x00, 0x00, 0x40, 0x00, 0x01, 0xdd, 0xfe,
};

static QueueHandle_t rx_queue;
static uint32_t sent = 0;
static uint32_t mismatches = 0;

// Build a frame straight from the recovered packet bytes so the test exercises the
// same bytes M3 will have to emit.
static bool send_bytes(uint32_t identifier, const uint8_t *bytes, uint8_t length) {
    twai_message_t message{};
    message.identifier = identifier;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = length;
    for (uint8_t index = 0; index < length; ++index) {
        message.data[index] = bytes[index];
    }
    return twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK;
}

static bool wait_for(uint32_t identifier, const uint8_t *data, uint8_t length,
                     uint32_t timeout_ms) {
    unsigned long deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        twai_message_t message{};
        if (twai_receive(&message, pdMS_TO_TICKS(20)) != ESP_OK) {
            continue;
        }
        if (message.identifier != identifier || message.data_length_code != length) {
            ++mismatches;
            continue;
        }
        if (memcmp(message.data, data, length) != 0) {
            ++mismatches;
            continue;
        }
        return true;
    }
    return false;
}

void setup() {
    digitalWrite(PIN_TX, HIGH);
    pinMode(PIN_TX, OUTPUT);
    Serial.begin(921600);
    delay(1000);
    Serial.println("# LOOPBACK SELF TEST - nothing may be attached to a real bus");
    Serial.printf("# tx_gpio=%u rx_gpio=%u bitrate=%u\n", PIN_TX, PIN_RX, CAN_BITRATE);

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TX, (gpio_num_t)PIN_RX, TWAI_MODE_NO_ACK);
    general.tx_queue_len = 8;
    general.rx_queue_len = 32;
    twai_timing_config_t timing = LOOPBACK_TIMING();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t result = twai_driver_install(&general, &timing, &filter);
    if (result == ESP_OK) {
        result = twai_start();
    }
    if (result != ESP_OK) {
        Serial.printf("# ERROR twai_start=%s\n", esp_err_to_name(result));
        return;
    }

    // Step 1: a plain standard frame at a known identifier.
    twai_message_t plain{};
    plain.identifier = 0x123;
    plain.data_length_code = 4;
    plain.data[0] = 0xDE; plain.data[1] = 0xAD; plain.data[2] = 0xBE; plain.data[3] = 0xEF;
    bool step1_sent = twai_transmit(&plain, pdMS_TO_TICKS(100)) == ESP_OK;
    bool step1_ok = step1_sent && wait_for(0x123, plain.data, 4, 500);
    Serial.printf("# STEP1 plain_frame sent=%u received_ok=%u\n", step1_sent, step1_ok);

    // Step 2: the exact 00:01 bring-up request split into its two real CAN frames.
    bool step2_first = send_bytes(0x503, BRINGUP_REQUEST, 8);
    bool step2_first_ok = wait_for(0x503, BRINGUP_REQUEST, 8, 500);
    bool step2_second = send_bytes(0x503, BRINGUP_REQUEST + 8, 5);
    bool step2_second_ok = wait_for(0x503, BRINGUP_REQUEST + 8, 5, 500);
    bool step2_ok = step2_first && step2_first_ok && step2_second && step2_second_ok;
    Serial.printf("# STEP2 bringup_request frames_sent=%u/%u received_ok=%u/%u ok=%u\n",
                  step2_first, step2_second, step2_first_ok, step2_second_ok, step2_ok);

    // Step 3: a burst, to expose queue or timing problems under load.
    uint32_t burst_ok = 0;
    const uint32_t burst_count = 100;
    for (uint32_t index = 0; index < burst_count; ++index) {
        twai_message_t message{};
        message.identifier = 0x300 + (index & 0x0F);
        message.data_length_code = 8;
        for (uint8_t byte = 0; byte < 8; ++byte) {
            message.data[byte] = (uint8_t)(index + byte);
        }
        if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
            ++sent;
            if (wait_for(message.identifier, message.data, 8, 200)) {
                ++burst_ok;
            }
        }
    }
    Serial.printf("# STEP3 burst sent=%u confirmed=%u/%u\n", sent, burst_ok, burst_count);

    twai_status_info_t status{};
    twai_get_status_info(&status);
    bool all_ok = step1_ok && step2_ok && burst_ok == burst_count;
    Serial.printf("# RESULT %s tx_errors=%u rx_missed=%u rx_overrun=%u bus_errors=%u "
                  "mismatches=%u\n",
                  all_ok ? "PASS" : "FAIL", status.tx_error_counter,
                  status.rx_missed_count, status.rx_overrun_count,
                  status.bus_error_count, mismatches);
    if (!all_ok) {
        Serial.println("# HINT bus must be empty (no motor, no RS3); the transceiver "
                       "loops TX back to RX internally, so no jumper is needed");
    }
    twai_stop();
    twai_driver_uninstall();
}

void loop() {
    delay(5000);
}
