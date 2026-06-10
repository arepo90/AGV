/* =============================================================================
 *  AGV USB-CDC ↔ UART bridge (ESP32-C3 SuperMini)
 *
 *  The ESP32 is a byte pump between the Jetson (over the ESP32-C3's native
 *  USB-CDC, exposed as `Serial`) and the STM32 (over UART1), plus two local
 *  duties: the indicator LEDs and the INA219 battery sensor.
 *
 *  Direction summary:
 *    USB-CDC (Jetson) ─►  UART (STM32)   byte pump + boundary tracker, so the
 *                                        locally-sourced PKT_BATTERY frames are
 *                                        spliced between host frames, never into one
 *    UART (STM32)     ─►  USB-CDC (Jetson) + parser tap for LED rings + status LED
 *
 *  No NACKing, no frame validation in either direction: the Jetson's bridge
 *  handles every protocol concern. Frame parsing is kept only to find frame
 *  boundaries and to drive the LED rings / onboard status LED from telemetry.
 *  The INA219 (3S bus voltage) is polled locally and pushed to the STM32,
 *  which keeps the low-voltage safety decision.
 * =============================================================================
 */

#include <Arduino.h>

#include "battery.h"
#include "config.h"
#include "crc.h"
#include "frame.h"
#include "ledring.h"
#include "status_led.h"

static HardwareSerial s_stm32(1);
static frame_parser_t s_uart_parser;
static frame_parser_t s_usb_parser;     /* boundary tracker for frame injection */
static uint32_t       s_usb_last_byte_ms = 0;

/* Tap completed UART frames to drive the local indicators. Frame layout:
 * index 4 = TYPE, index 5 = LEN, index 6+ = payload.
 *   CORE (0x03):    mode/function at payload off 4/5, estop/caution u16 at 6/8,
 *                   proximity u16 at 39, led_mode at 41, led_indicator_cfg at 42.
 *   SENSORS (0x0A): load_cells f32[4] (0..15), batt_3s u16 (16..17),
 *                   LiDAR segment tail u16 from off 18. */
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
        if (len < 18) return;
        uint8_t nseg = (uint8_t)((len - 18u) / 2u);
        if (nseg > LED_LIDAR_MAX_SEGMENTS) nseg = LED_LIDAR_MAX_SEGMENTS;
        uint16_t lidar[LED_LIDAR_MAX_SEGMENTS];
        for (uint8_t k = 0; k < nseg; k++)
            lidar[k] = (uint16_t)(pl[18 + 2 * k] | (pl[18 + 2 * k + 1] << 8));

        ledring_update_sensors(lidar, nseg);
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

/* ---- USB → UART: byte pump + frame-boundary tracker ---------------------- */
static void pump_usb_to_uart(void) {
    while (Serial.available()) {
        uint8_t buf[256];
        int avail = Serial.available();
        if (avail > (int)sizeof(buf)) avail = sizeof(buf);
        int n = Serial.readBytes(buf, avail);
        if (n <= 0) break;
        s_stm32.write(buf, n);
        for (int i = 0; i < n; i++) frame_parser_feed(&s_usb_parser, buf[i]);
        s_usb_last_byte_ms = millis();
    }
}

/* ---- Local battery → STM32 frame injection ------------------------------- */

static bool     s_batt_pending = false;
static uint16_t s_batt_mv = 0;
static uint8_t  s_inject_seq = 0;

/* Build + send one ESP32-sourced frame to the STM32. Own SEQ stream: the
 * firmware only uses inbound SEQ to echo in ACKs, and PKT_BATTERY is
 * fire-and-forget, so interleaving with the host's SEQ is harmless. */
static void send_to_stm32(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t f[PROTO_MAX_FRAME];
    f[0] = PROTO_MAGIC0;
    f[1] = PROTO_MAGIC1;
    f[2] = PROTO_VERSION;
    f[3] = s_inject_seq++;
    f[4] = type;
    f[5] = len;
    for (uint8_t i = 0; i < len; i++) f[6 + i] = payload[i];
    uint16_t crc = crc16_compute(&f[2], 4u + (uint32_t)len);
    f[6 + len] = (uint8_t)(crc & 0xFF);
    f[7 + len] = (uint8_t)(crc >> 8);
    s_stm32.write(f, 8u + len);
}

/* Inject when the host stream sits at a frame boundary. If it stalls mid-frame
 * longer than BATT_INJECT_FORCE_MS (host died / desynced), inject anyway: the
 * STM32 CRC-rejects the orphaned half-frame and resyncs on our MAGIC, whereas
 * holding off would silently starve the battery monitor into staleness. */
static void inject_battery(uint32_t now) {
    if (!s_batt_pending) return;
    if (s_usb_parser.state != frame_parser_t::S_MAGIC0) {
        if ((uint32_t)(now - s_usb_last_byte_ms) < BATT_INJECT_FORCE_MS) return;
        frame_parser_init(&s_usb_parser, nullptr, nullptr);
    }
    uint8_t p[2] = { (uint8_t)(s_batt_mv & 0xFF), (uint8_t)(s_batt_mv >> 8) };
    send_to_stm32(PKT_BATTERY, p, sizeof p);
    s_batt_pending = false;
}

/* ---- setup / loop ------------------------------------------------------- */
void setup() {
    /* USB-CDC `Serial` carries the binary protocol — no debug prints allowed.
     * Begin with a baud rate hint for hosts that care; CDC ignores it. */
    Serial.begin(921600);

    ledring_init();
    status_led_init(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW);
    battery_init();

    s_stm32.setRxBufferSize(UART_RX_BUFSIZE);
    s_stm32.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    frame_parser_init(&s_uart_parser, on_complete, on_error);
    frame_parser_init(&s_usb_parser, nullptr, nullptr);
}

void loop() {
    pump_uart_to_usb();
    pump_usb_to_uart();
    uint32_t now = millis();
    uint16_t mv;
    if (battery_poll(now, &mv)) { s_batt_mv = mv; s_batt_pending = true; }
    inject_battery(now);
    ledring_tick(now);
    status_led_tick(now);
}
