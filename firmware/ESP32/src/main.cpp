/* =============================================================================
 *  AGV USB-CDC ↔ UART bridge (ESP32-C3 SuperMini)
 *
 *  The ESP32 is now a transparent byte pump between the Jetson (over the
 *  ESP32-C3's native USB-CDC, exposed as `Serial`) and the STM32 (over UART1).
 *
 *  Direction summary:
 *    USB-CDC (Jetson) ─►  UART (STM32)   pure byte pump, no parsing
 *    UART (STM32)     ─►  USB-CDC (Jetson) + parser tap for LED rings + status LED
 *
 *  No WS, no Wi-Fi, no NACKing, no frame validation in either direction:
 *  the Jetson's bridge handles every protocol concern. Frame parsing is
 *  kept only to drive the stack-light and onboard status LED from
 *  telemetry — that visual feedback is the ESP32's only remaining duty
 *  besides byte forwarding.
 * =============================================================================
 */

#include <Arduino.h>

#include "config.h"
#include "frame.h"
#include "ledring.h"
#include "status_led.h"

static HardwareSerial s_stm32(1);
static frame_parser_t s_uart_parser;

/* Tap completed UART frames to drive the local indicators. Frame layout:
 * index 4 = TYPE, index 5 = LEN, index 6+ = payload.
 *   CORE (0x03):    mode/function at payload off 4/5, estop/caution u16 at 6/8,
 *                   proximity u16 at 39, led_mode at 41, led_indicator_cfg at 42.
 *   SENSORS (0x0A): tof_mm[4] u16 from off 29; LiDAR segment tail u16 from off 41. */
static void tap_telemetry(const uint8_t *frame, size_t total_len) {
    if (total_len < 8) return;
    uint8_t        type = frame[4];
    uint8_t        len  = frame[5];
    const uint8_t *pl   = &frame[6];

    if (type == 0x03 /* PKT_TLM_CORE */) {
        if (len < 43) return;       /* need through led_indicator_cfg at offset 42 */
        uint8_t  mode            = pl[4];
        uint8_t  function        = pl[5];
        uint16_t estop_sources   = (uint16_t)(pl[6] | (pl[7] << 8));
        uint16_t caution_sources = (uint16_t)(pl[8] | (pl[9] << 8));
        uint16_t proximity_bits  = (uint16_t)(pl[39] | (pl[40] << 8));
        uint8_t  led_mode        = pl[41];
        uint8_t  indicator_cfg   = pl[42];

        ledring_update_from_telemetry(estop_sources, caution_sources, led_mode,
                                      indicator_cfg, proximity_bits, millis());
        status_led_update_estop(estop_sources != 0);
        status_led_update_mode(mode == 0);          /* mode 0 = SUPERVISED */
        status_led_update_function(function);
    } else if (type == 0x0A /* PKT_TLM_SENSORS */) {
        if (len < 41) return;
        uint16_t tof[4];
        for (int i = 0; i < 4; i++)
            tof[i] = (uint16_t)(pl[29 + 2 * i] | (pl[29 + 2 * i + 1] << 8));

        uint8_t nseg = (uint8_t)((len - 41u) / 2u);
        if (nseg > LED_LIDAR_MAX_SEGMENTS) nseg = LED_LIDAR_MAX_SEGMENTS;
        uint16_t lidar[LED_LIDAR_MAX_SEGMENTS];
        for (uint8_t k = 0; k < nseg; k++)
            lidar[k] = (uint16_t)(pl[41 + 2 * k] | (pl[41 + 2 * k + 1] << 8));

        ledring_update_sensors(tof, lidar, nseg);
    }
}

static void on_complete(const uint8_t *frame, size_t total_len) {
    tap_telemetry(frame, total_len);
}

static void on_error(frame_result_t /*err*/, uint8_t /*suspect_seq*/) {
    /* Bytes still pass through to USB unchanged; the Jetson's parser will
     * notice the CRC mismatch and log it. Parser-side errors here only mean
     * "this byte stream desynced from frame boundaries" — we resync silently. */
}

/* ---- UART → USB: stream bytes through; feed tap parser alongside -------- */
static void pump_uart_to_usb(void) {
    while (s_stm32.available()) {
        /* Pull up to 256 bytes per loop iteration to keep both pumps moving. */
        uint8_t buf[256];
        int avail = s_stm32.available();
        if (avail > (int)sizeof(buf)) avail = sizeof(buf);
        int n = s_stm32.readBytes(buf, avail);
        if (n <= 0) break;
        Serial.write(buf, n);
        for (int i = 0; i < n; i++) frame_parser_feed(&s_uart_parser, buf[i]);
    }
}

/* ---- USB → UART: pure byte pump ----------------------------------------- */
static void pump_usb_to_uart(void) {
    while (Serial.available()) {
        uint8_t buf[256];
        int avail = Serial.available();
        if (avail > (int)sizeof(buf)) avail = sizeof(buf);
        int n = Serial.readBytes(buf, avail);
        if (n <= 0) break;
        s_stm32.write(buf, n);
    }
}

/* ---- setup / loop ------------------------------------------------------- */
void setup() {
    /* USB-CDC `Serial` carries the binary protocol — no debug prints allowed.
     * Begin with a baud rate hint for hosts that care; CDC ignores it. */
    Serial.begin(921600);

    ledring_init();
    status_led_init(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW);

    s_stm32.setRxBufferSize(UART_RX_BUFSIZE);
    s_stm32.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    frame_parser_init(&s_uart_parser, on_complete, on_error);
}

void loop() {
    pump_uart_to_usb();
    pump_usb_to_uart();
    uint32_t now = millis();
    ledring_tick(now);
    status_led_tick(now);
}
