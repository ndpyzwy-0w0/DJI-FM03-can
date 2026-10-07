#include <Arduino.h>
#include <atomic>
#include "driver/twai.h"
#include "esp_timer.h"

struct CapturedFrame {
    int64_t timestamp_us;
    uint32_t sequence;
    twai_message_t frame;
};

static QueueHandle_t log_queue;
static std::atomic<uint32_t> received{0};
static std::atomic<uint32_t> logger_dropped{0};
static bool running = false;

// Timestamp is taken on driver dequeue, not at the physical frame edge.
static void receive_frames(void *) {
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

static void print_status() {
    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        Serial.println("# ERROR status_unavailable");
        return;
    }
    Serial.printf("# STATUS t_us=%llu bitrate=%u rx=%u logger_dropped=%u "
                  "rx_missed=%u rx_overrun=%u bus_errors=%u pending=%u "
                  "logger_pending=%u\n",
                  (unsigned long long)esp_timer_get_time(), CAN_BITRATE,
                  received.load(), logger_dropped.load(),
                  status.rx_missed_count, status.rx_overrun_count,
                  status.bus_error_count, status.msgs_to_rx,
                  (unsigned)uxQueueMessagesWaiting(log_queue));
}

void setup() {
    // Hold TX recessive before enabling TWAI. No CAN transmit API is used.
    digitalWrite(4, HIGH);
    pinMode(4, OUTPUT);
    Serial.begin(921600);
    delay(1000);
    Serial.printf("# START listen_only bitrate=%u tx_gpio=4 rx_gpio=5\n", CAN_BITRATE);
    Serial.println("# FRAME t_us,sequence,id_hex,extended,rtr,dlc,data_hex");
    Serial.println("# timestamp=driver_dequeue_us_since_boot; command=s_status");

    log_queue = xQueueCreate(1024, sizeof(CapturedFrame));
    if (!log_queue) {
        Serial.println("# ERROR log_queue_allocation");
        return;
    }
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_LISTEN_ONLY);
    general.tx_queue_len = 0;
    general.rx_queue_len = 512;
#if CAN_BITRATE == 1000000
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BITRATE == 500000
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
#else
#error Unsupported CAN bitrate
#endif
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t result = twai_driver_install(&general, &timing, &filter);
    if (result == ESP_OK) result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("# ERROR twai_start=%s\n", esp_err_to_name(result));
        return;
    }
    if (xTaskCreatePinnedToCore(receive_frames, "can_rx", 4096, nullptr,
                              10, nullptr, 0) != pdPASS) {
        twai_stop();
        Serial.println("# ERROR receive_task_allocation");
        return;
    }
    running = true;
    Serial.println("# READY listen_only");
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
    }
    static unsigned long last_status = 0;
    bool requested = false;
    while (Serial.available()) {
        if (Serial.read() == 's') requested = true;
    }
    if (requested || millis() - last_status >= 1000) {
        print_status();
        last_status = millis();
    }
}
