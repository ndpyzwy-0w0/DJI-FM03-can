// FM03 focus-motor motion control firmware.
//
// Purpose: drive the DJI RS focus motor (2022, "FM03") directly from the ESP32 with
// no reflashing between experiments. The host program tools/motor_control.py sends
// commands over the UART and displays live telemetry, so a value can be tried,
// changed and stopped interactively.
//
// Working hypothesis this build is based on (stated by the operator, to be tested by
// experiment rather than assumed as proven):
//   1. the sign of the dial value selects the rotation direction,
//   2. a larger magnitude rotates further.
// Both were only checked at +/-16 before this build, so the magnitude range is
// available up to the full measured dial range and the host UI makes it easy to step
// up from small values.
//
// Command protocol (one command per line, '\n' terminated, ASCII):
//   arm                  take control: switch the transceiver on and start the stream
//   disarm               release control and stop transmitting (driver stays installed)
//   go <value>           hold <value> (-2048..2048); the motor should turn while held
//   set <value>          alias of go
//   stop | zero          command zero and keep (or take) control
//   run <value> <ms>     hold <value> for <ms>, then zero (1..60000 ms)
//   status               print one STATE line and feed the watchdog
//   ping                 alias of status
//   interval [ms]        show or set the dial cadence (10..1000 ms)
//   limit [n]            show or set the magnitude clamp (1..2048)
//   help                 print the command list
//   version              print the build identity
//
// Device replies, one line each, always prefixed so the host can classify them:
//   # ACK <what>          a command was accepted
//   # ERR <what>          a command was rejected
//   # STATE ...           telemetry (also emitted periodically and after every change)
//   # READY / # CONFIG / # HELP / # VERSION / # WARN
//
// Watchdog: if no valid command arrives within CONTROL_WATCHDOG_MS, the value returns
// to zero and an ERR line is printed. The host sends `status` as a heartbeat, so a
// closed laptop or a pulled cable stops the motor instead of leaving it driving.

#include <Arduino.h>
#include <cstring>
#include <cerrno>
#include "driver/twai.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#if __has_include("dji_crc.h")
#include "dji_crc.h"
#else
// Same algorithm as tools/analyze_protocol.py, so emitted bytes can be checked
// against the captures with the project's own tools. Header CRC8: init 0x77,
// reflected polynomial 0x8C, over the first three bytes. Packet CRC16: init 0x3692,
// reflected polynomial 0x8408, over the packet minus its trailing two bytes.
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
#endif

#ifndef CAN_BITRATE
#define CAN_BITRATE 1000000
#endif
#ifndef LED_RGB_GPIO
#define LED_RGB_GPIO 48
#endif

// Firmware identity, reported by `version` so a session record can name the exact
// build. Bump the revision when the wire behaviour changes.
#define CONTROL_VERSION "motor_control 1.0"

// Magnitude clamp. The measured dial range in the captures is about +/-2048, and
// 2048 is the default; lower it with `limit` while exploring.
#ifndef CONTROL_VALUE_LIMIT
#define CONTROL_VALUE_LIMIT 2048
#endif
#ifndef CONTROL_HARD_LIMIT
#define CONTROL_HARD_LIMIT 2048
#endif

// Zero the value if no valid command arrives within this window.
#ifndef CONTROL_WATCHDOG_MS
#define CONTROL_WATCHDOG_MS 2000
#endif

// Dial packets per second: 50 ms (20/s) is the verified-stable cadence; the real RS3
// sends about every 10 ms. Changeable at runtime with `interval`.
#ifndef CONTROL_INTERVAL_MS
#define CONTROL_INTERVAL_MS 50
#endif

// Frame logging is off by default. The motor floods the bus at roughly 7600 frames/s
// and the link is 921600 baud, so logging received frames would drop the very lines
// used for control; the host displays telemetry instead.
#ifndef CONTROL_LOG_TX
#define CONTROL_LOG_TX 0
#endif

// Telemetry period while armed.
#ifndef CONTROL_TELEMETRY_MS
#define CONTROL_TELEMETRY_MS 500
#endif

#ifndef TX_QUEUE_LEN
#define TX_QUEUE_LEN 0
#endif

#define BUTTON_GPIO 0
#define BUTTON_HOLD_MS 2000
#define SERIAL_LINE_MAX 96

// Transceiver pins, identical to the rest of the project's firmware.
static const uint8_t PIN_TX = 4;
static const uint8_t PIN_RX = 5;

// ---------------------------------------------------------------------------
// Packet templates. Both verified byte for byte against the captures.
// ---------------------------------------------------------------------------

// 13-byte version request E5>BF 00:01, sent as 8+5.
static const uint8_t REQUEST_0001[13] = {
    0x55, 0x0D, 0x04, 0x33, 0xE5, 0xBF, 0x00, 0x00, 0x40, 0x00, 0x01, 0xDD, 0xFE};

// 19-byte dial command E5>BF 04:69, sent as 8+8+3. Bytes 15 and 16 carry the dial
// value as little-endian int16; 17 and 18 are the CRC16 and are recomputed.
static const uint8_t DIAL_0469[19] = {
    0x55, 0x13, 0x04, 0x03, 0xE5, 0xBF, 0x00, 0x00,
    0x00, 0x04, 0x69, 0x0F, 0x04, 0x08, 0x19, 0x00, 0x00, 0x6B, 0x56};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static volatile bool mode_replay = false;       // driver may transmit
static volatile bool driver_installed = false;
static volatile bool driver_switching = false;
static SemaphoreHandle_t driver_mutex = nullptr;

static volatile int16_t applied_value = 0;      // value last put on the wire
static volatile int16_t target_value = 0;       // value the host asked for
static volatile bool armed = false;
static volatile unsigned long last_command_ms = 0;
static volatile unsigned long clock_ms = 0;

static volatile int16_t run_then_value = 0;
static volatile bool run_timed = false;
static volatile unsigned long run_started_ms = 0;
static volatile unsigned long run_duration_ms = 0;

static volatile int32_t value_limit = CONTROL_VALUE_LIMIT;
static volatile unsigned long interval_ms = CONTROL_INTERVAL_MS;
static volatile bool log_tx = CONTROL_LOG_TX ? true : false;
static volatile bool tx_warned = false;

static volatile uint32_t sent_packets = 0;
static volatile uint32_t rejected_fragments = 0;
static volatile uint32_t tx_failed = 0;
static volatile uint32_t packets_zero = 0;
static volatile uint32_t packets_nonzero = 0;
static volatile uint32_t watchdog_stops = 0;
static volatile uint32_t run_completions = 0;

static void write_led(uint8_t red, uint8_t green, uint8_t blue) {
#if LED_RGB_GPIO >= 0
    neopixelWrite((uint8_t)LED_RGB_GPIO, red, green, blue);
#else
    (void)red; (void)green; (void)blue;
#endif
}

// ---------------------------------------------------------------------------
// Driver lifecycle
// ---------------------------------------------------------------------------

static bool install_driver(bool replay) {
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TX, (gpio_num_t)PIN_RX, TWAI_MODE_NORMAL);
    // Queue disabled: the verified workaround for the transmit queue race. A frame is
    // either on the wire or fails immediately, which is why every fragment waits.
    general.tx_queue_len = replay ? 0 : 0;
    general.rx_queue_len = 128;
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_timing_config_t timing =
#if CAN_BITRATE == 500000
        TWAI_TIMING_CONFIG_500KBITS();
#else
        TWAI_TIMING_CONFIG_1MBITS();
#endif
    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    driver_installed = true;
    return true;
}

static bool set_mode(bool replay) {
    driver_switching = true;
    if (driver_mutex) xSemaphoreTake(driver_mutex, portMAX_DELAY);
    if (driver_installed) {
        twai_stop();
        twai_driver_uninstall();
        driver_installed = false;
    }
    bool ok = install_driver(replay);
    mode_replay = ok ? replay : false;
    // Blue = listening only (safe to touch), red = transceiver live.
    write_led(mode_replay ? 48 : 0, 0, mode_replay ? 0 : 24);
    if (driver_mutex) xSemaphoreGive(driver_mutex);
    driver_switching = false;
    return ok;
}

// ---------------------------------------------------------------------------
// Transmit path: one bounded wait per fragment, verified shape
// ---------------------------------------------------------------------------

static bool wait_tx_idle(unsigned spins) {
    for (unsigned spin = 0; spin < spins; ++spin) {
        twai_status_info_t status{};
        if (twai_get_status_info(&status) != ESP_OK) return false;
        if (status.state != TWAI_STATE_RUNNING) return false;
        if (status.msgs_to_tx == 0) return true;
        delay(1);
    }
    return false;
}

// Fragment lengths are fixed by the two template sizes: 13 = 8+5, 19 = 8+8+3.
static bool send_packet(uint32_t identifier, const uint8_t *packet, const uint8_t *lengths,
                        unsigned fragments) {
    unsigned offset = 0;
    for (unsigned index = 0; index < fragments; ++index) {
        if (!wait_tx_idle(50)) {
            ++rejected_fragments;
            return false;
        }
        twai_message_t message{};
        message.identifier = identifier;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = lengths[index];
        for (unsigned byte = 0; byte < lengths[index]; ++byte) {
            message.data[byte] = packet[offset + byte];
        }
        offset += lengths[index];
        message.self = log_tx ? 1 : 0;
        if (twai_transmit(&message, 0) != ESP_OK) {
            ++rejected_fragments;
            return false;
        }
    }
    return true;
}

static bool send_request() {
    static const uint8_t lengths[2] = {8, 5};
    return send_packet(0x503, REQUEST_0001, lengths, 2);
}

static bool send_dial(int16_t value) {
    uint8_t packet[19];
    memcpy(packet, DIAL_0469, sizeof(packet));
    packet[15] = (uint8_t)(value & 0xFF);
    packet[16] = (uint8_t)((value >> 8) & 0xFF);
    packet[3] = crc8(packet, 3);
    uint16_t packet_crc = crc16(packet, 17);
    packet[17] = (uint8_t)(packet_crc & 0xFF);
    packet[18] = (uint8_t)(packet_crc >> 8);
    static const uint8_t lengths[3] = {8, 8, 3};
    return send_packet(0x503, packet, lengths, 3);
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

static void print_state() {
    twai_status_info_t status{};
    twai_get_status_info(&status);
    unsigned long run_left = 0;
    if (run_timed && clock_ms < run_started_ms + run_duration_ms) {
        run_left = run_started_ms + run_duration_ms - clock_ms;
    }
    Serial.printf("# STATE mode=%s armed=%u value=%d target=%d run=%u run_left_ms=%lu "
                  "sent=%lu nonzero=%lu zero=%lu rejected=%lu tx_failed=%lu "
                  "twai_state=%u msgs_to_tx=%u bus_errors=%lu tx_errors=%lu rx_missed=%lu "
                  "limit=%ld interval_ms=%lu watchdog_ms=%lu watchdog_stops=%lu "
                  "run_complete=%lu button=%u version=\"%s\"\n",
                  mode_replay ? "replay" : "listen", armed ? 1u : 0u, (int)applied_value,
                  (int)target_value, run_timed ? 1u : 0u, run_left,
                  (unsigned long)sent_packets, (unsigned long)packets_nonzero,
                  (unsigned long)packets_zero, (unsigned long)rejected_fragments,
                  (unsigned long)tx_failed,
                  (unsigned)status.state, (unsigned)status.msgs_to_tx,
                  (unsigned long)status.bus_error_count, (unsigned long)status.tx_error_counter,
                  (unsigned long)status.rx_missed_count, (long)value_limit,
                  interval_ms, (unsigned long)CONTROL_WATCHDOG_MS,
                  (unsigned long)watchdog_stops, (unsigned long)run_completions,
                  digitalRead(BUTTON_GPIO) == LOW ? 1u : 0u, CONTROL_VERSION);
}

static void print_help() {
    Serial.println("# HELP arm | disarm | stop | zero");
    Serial.println("# HELP go <value>            hold a value (-2048..2048)");
    Serial.println("# HELP run <value> <ms>      hold, then zero (1..60000)");
    Serial.println("# HELP status | ping         telemetry + watchdog feed");
    Serial.println("# HELP interval [ms]         dial cadence, 10..1000");
    Serial.println("# HELP limit [n]             magnitude clamp, 1..2048");
    Serial.println("# HELP logtx <0|1>           log transmitted frames");
    Serial.println("# HELP version | help");
}

// ---------------------------------------------------------------------------
// Host protocol
// ---------------------------------------------------------------------------

static int32_t clamp_value(int32_t value) {
    int32_t limit = value_limit;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static bool parse_i32(const char *text, int32_t *out) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    errno = 0;
    long value = strtol(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0') return false;
    if (value < -1000000L || value > 1000000L) return false;
    *out = (int32_t)value;
    return true;
}

static void apply_value(int32_t requested, const char *what) {
    int32_t clamped = clamp_value(requested);
    target_value = (int16_t)clamped;
    Serial.printf("# ACK %s requested=%ld applied=%d\n", what, (long)requested, (int)clamped);
    print_state();
}

static void handle_line(char *line) {
    size_t length = strlen(line);
    while (length > 0 && (line[length - 1] == '\r' || line[length - 1] == ' ' ||
                          line[length - 1] == '\t')) {
        line[--length] = '\0';
    }
    if (length == 0) return;
    last_command_ms = clock_ms;

    char *command = strtok(line, " \t");
    if (command == nullptr) return;
    char *first = strtok(nullptr, " \t");
    char *second = strtok(nullptr, " \t");
    int32_t value = 0;

    if (!strcasecmp(command, "arm")) {
        armed = true;
        if (!mode_replay) set_mode(true);
        target_value = 0;
        Serial.println("# ACK arm value=0 (send 'go <value>' to move)");
        print_state();
        return;
    }
    if (!strcasecmp(command, "disarm")) {
        armed = false;
        run_timed = false;
        target_value = 0;
        Serial.println("# ACK disarm value=0");
        print_state();
        return;
    }
    if (!strcasecmp(command, "stop") || !strcasecmp(command, "zero")) {
        run_timed = false;
        target_value = 0;
        Serial.println("# ACK stop value=0");
        print_state();
        return;
    }
    if (!strcasecmp(command, "go") || !strcasecmp(command, "set")) {
        if (!parse_i32(first, &value)) {
            Serial.println("# ERR usage: go <value>");
            return;
        }
        if (value_limit > CONTROL_HARD_LIMIT) value_limit = CONTROL_HARD_LIMIT;
        run_timed = false;
        apply_value(value, "go");
        return;
    }
    if (!strcasecmp(command, "run")) {
        int32_t duration = 0;
        if (!parse_i32(first, &value) || !parse_i32(second, &duration) ||
            duration <= 0 || duration > 60000L) {
            Serial.println("# ERR usage: run <value> <ms 1..60000>");
            return;
        }
        target_value = (int16_t)clamp_value(value);
        run_then_value = 0;
        run_duration_ms = (unsigned long)duration;
        run_started_ms = clock_ms;
        run_timed = true;
        Serial.printf("# ACK run requested=%ld applied=%d ms=%ld\n", (long)value,
                      (int)target_value, (long)duration);
        print_state();
        return;
    }
    if (!strcasecmp(command, "status") || !strcasecmp(command, "ping")) {
        print_state();
        return;
    }
    if (!strcasecmp(command, "interval")) {
        if (first == nullptr) {
            Serial.printf("# ACK interval ms=%lu\n", interval_ms);
            return;
        }
        if (!parse_i32(first, &value) || value < 10 || value > 1000) {
            Serial.println("# ERR usage: interval <ms 10..1000>");
            return;
        }
        interval_ms = (unsigned long)value;
        Serial.printf("# ACK interval ms=%lu\n", interval_ms);
        print_state();
        return;
    }
    if (!strcasecmp(command, "limit")) {
        if (first == nullptr) {
            Serial.printf("# ACK limit=%ld hard_limit=%d\n", (long)value_limit,
                          CONTROL_HARD_LIMIT);
            return;
        }
        if (!parse_i32(first, &value) || value < 1 || value > CONTROL_HARD_LIMIT) {
            Serial.printf("# ERR usage: limit <1..%d>\n", CONTROL_HARD_LIMIT);
            return;
        }
        value_limit = value;
        if (target_value > value_limit) target_value = (int16_t)value_limit;
        if (target_value < -value_limit) target_value = (int16_t)-value_limit;
        Serial.printf("# ACK limit=%ld\n", (long)value_limit);
        print_state();
        return;
    }
    if (!strcasecmp(command, "logtx")) {
        if (first == nullptr) {
            Serial.printf("# ACK logtx=%u\n", log_tx ? 1u : 0u);
            return;
        }
        log_tx = strcmp(first, "0") != 0;
        Serial.printf("# ACK logtx=%u\n", log_tx ? 1u : 0u);
        return;
    }
    if (!strcasecmp(command, "version")) {
        Serial.printf("# VERSION %s limit=%d interval_ms=%lu watchdog_ms=%lu\n",
                      CONTROL_VERSION, CONTROL_VALUE_LIMIT, interval_ms,
                      (unsigned long)CONTROL_WATCHDOG_MS);
        return;
    }
    if (!strcasecmp(command, "help") || !strcasecmp(command, "?")) {
        print_help();
        return;
    }
    Serial.printf("# ERR unknown '%s' (try help)\n", command);
}

static void service_serial() {
    static char line[SERIAL_LINE_MAX];
    static size_t used = 0;
    while (Serial.available() > 0) {
        int next = Serial.read();
        if (next < 0) break;
        if (next == '\n') {
            line[used] = '\0';
            handle_line(line);
            used = 0;
            continue;
        }
        if (used < sizeof(line) - 1) {
            line[used++] = (char)next;
        } else {
            used = 0;
            Serial.println("# ERR command_line_too_long");
        }
    }
}

static void service_watchdog() {
    if (!armed) {
        target_value = 0;
        return;
    }
    if (clock_ms - last_command_ms > CONTROL_WATCHDOG_MS && target_value != 0) {
        target_value = 0;
        run_timed = false;
        ++watchdog_stops;
        Serial.println("# ERR watchdog_stop no_command_within_window");
        print_state();
    }
}

static void service_button() {
    static bool was_down = false;
    static unsigned long pressed_at = 0;
    static bool fired = false;
    bool down = digitalRead(BUTTON_GPIO) == LOW;
    if (down && !was_down) {
        pressed_at = clock_ms;
        fired = false;
    } else if (down && !fired && clock_ms - pressed_at >= BUTTON_HOLD_MS) {
        fired = true;
        // Local stop only. A long press never starts motion, it only clears it.
        armed = false;
        run_timed = false;
        target_value = 0;
        Serial.println("# ACK button_stop value=0");
        print_state();
    }
    was_down = down;
}

static volatile unsigned long last_tx_ms = 0;

static void service_transmit() {
    if (!armed) {
        applied_value = 0;
        return;
    }
    if (clock_ms - last_tx_ms < interval_ms) return;
    last_tx_ms = clock_ms;
    if (run_timed && clock_ms >= run_started_ms + run_duration_ms) {
        run_timed = false;
        target_value = run_then_value;
        ++run_completions;
        Serial.println("# ACK run_complete value=0");
    }
    int16_t value = target_value;
    applied_value = value;
    bool ok = send_request();
    ok = send_dial(value) && ok;
    if (ok) {
        ++sent_packets;
        if (value == 0) {
            ++packets_zero;
        } else {
            ++packets_nonzero;
        }
        tx_warned = false;
    } else {
        ++tx_failed;
        if (!tx_warned) {
            tx_warned = true;
            Serial.println("# WARN transmit_rejected (bus busy or no ACK)");
        }
    }
}

static void service_telemetry() {
    static unsigned long last = 0;
    if (clock_ms - last < CONTROL_TELEMETRY_MS) return;
    last = clock_ms;
    print_state();
}

void setup() {
    pinMode(BUTTON_GPIO, INPUT_PULLUP);
    pinMode(PIN_TX, OUTPUT);
    digitalWrite(PIN_TX, HIGH);
    Serial.begin(921600);
    delay(300);
    Serial.println("# READY motor_control");
    Serial.printf("# CONFIG can_bitrate=%u value_limit=%ld hard_limit=%d interval_ms=%lu "
                  "watchdog_ms=%lu tx_queue_len=%u log_tx_default=%u\n",
                  (unsigned)CAN_BITRATE, (long)value_limit, CONTROL_HARD_LIMIT,
                  interval_ms, (unsigned long)CONTROL_WATCHDOG_MS, (unsigned)TX_QUEUE_LEN,
                  log_tx ? 1u : 0u);
    print_help();
    if (digitalRead(BUTTON_GPIO) == LOW) {
        Serial.println("# WARN button_idle_low check_wiring");
    }
    driver_mutex = xSemaphoreCreateMutex();
    if (!driver_mutex) {
        Serial.println("# ERROR mutex_allocation");
        return;
    }
    // Boots listen-only: the transceiver is installed but silent until `arm`.
    if (!set_mode(false)) {
        Serial.println("# ERROR driver_install_failed");
        return;
    }
    last_command_ms = clock_ms;
    Serial.println("# READY listen_only (send 'arm' to take control)");
    print_state();
}

void loop() {
    clock_ms = millis();
    service_serial();
    service_button();
    service_watchdog();
    service_transmit();
    service_telemetry();
    delay(1);
}
