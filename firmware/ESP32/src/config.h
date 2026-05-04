#pragma once

/* =============================================================================
 *  AGV ESP32-C3 relay configuration.
 *
 *  Mirrors the protocol-side constants from STM32/inc/config.h. Anything that
 *  changes on one side MUST change on the other — including PROTO_VERSION.
 * =============================================================================
 */

/* ---- Wi-Fi AP ------------------------------------------------------------- */
#define AP_SSID                 "AGV-relay"
#define AP_PASS                 "agvControl"     /* min 8 chars for WPA2 */
#define AP_CHANNEL              6

/* ---- UART link to STM32 --------------------------------------------------- */
#define UART_BAUD               921600u
#define UART_RX_PIN             5            /* connected to STM32 PB6 (TX) */
#define UART_TX_PIN             4            /* connected to STM32 PB7 (RX) */
#define UART_RX_BUFSIZE         1024u

/* ---- Packet protocol (must match STM32 exactly) -------------------------- */
#define PROTO_MAGIC0            0xAAu
#define PROTO_MAGIC1            0x56u
#define PROTO_VERSION           0x01u
#define PROTO_MAX_PAYLOAD       255u
#define PROTO_FRAME_OVERHEAD    8u
#define PROTO_MAX_FRAME         (PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD)

/* ---- Packet types (subset — relay only inspects header for routing) ------ */
#define PKT_NACK                0x06u

/* ---- NACK error codes ---------------------------------------------------- */
#define NACK_BAD_CRC            0x01u
#define NACK_BAD_LENGTH         0x02u
#define NACK_BAD_VERSION        0x03u

/* ---- Relay behavior ------------------------------------------------------ */
#define WS_PATH                 "/ws"
#define WS_MAX_CLIENTS          1            /* one workstation at a time */
#define DEBUG_LOG_BAD_FRAMES    1            /* Serial.printf on parse errors */

/* ---- Stack light + buzzer ----------------------------------------------- */
/* Wired through a 4-channel low-side MOSFET module on GPIO 0..3.
 * Default mapping: red → 0, yellow → 1, green → 2, buzzer → 3. Swap freely. */
#define STACK_PIN_RED           0
#define STACK_PIN_YELLOW        1
#define STACK_PIN_GREEN         2
#define STACK_PIN_BUZZER        3

#define STACK_LEDC_FREQ_HZ      1000u
#define STACK_LEDC_RESOLUTION   10u       /* 1024 levels */

#define STACK_TELEM_TIMEOUT_MS  2000u     /* no telemetry → DISCONNECT */
