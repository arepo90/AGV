#pragma once

/* =============================================================================
 *  AGV ESP32-C3 USB-CDC ↔ UART bridge configuration.
 *
 *  Mirrors protocol-side constants from STM32/inc/config.h. Anything that
 *  changes on one side MUST change on the other — including PROTO_VERSION.
 * =============================================================================
 */

/* ---- UART link to STM32 --------------------------------------------------- */
#define UART_BAUD               921600u
#define UART_RX_PIN             21           /* connected to STM32 PB6 (TX) */
#define UART_TX_PIN             20           /* connected to STM32 PB7 (RX) */
#define UART_RX_BUFSIZE         2048u         /* headroom for the LED show() blocking window */

/* ---- Packet protocol (must match STM32 exactly) -------------------------- */
#define PROTO_MAGIC0            0xAAu
#define PROTO_MAGIC1            0x56u
#define PROTO_VERSION           0x02u   /* v2: streamed telemetry (CORE/DRIVE/SENSORS/QTR) */
#define PROTO_MAX_PAYLOAD       255u
#define PROTO_FRAME_OVERHEAD    8u
#define PROTO_MAX_FRAME         (PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD)

/* ---- Status LED (onboard ESP32-C3 SuperMini) ----------------------------- */
/* GPIO 8, active-low (LOW = LED on). LED_BUILTIN resolves to an invalid
 * pseudo-pin on this variant (SOC_GPIO_PIN_COUNT + PIN_NEOPIXEL), so hardcode. */
#define STATUS_LED_PIN          8
#define STATUS_LED_ACTIVE_LOW   1

/* ---- Indicator LED rings (WS2812B) -------------------------------------- */
/* Four rings, all showing the same state in the stack-light colours
 * (green / yellow / red). Each ring is wired as a loop (first and last LED
 * adjacent) on its own data GPIO. Two large rings + two small; lengths differ
 * but the animation uses a normalised phase so the rings stay visually in sync.
 * Animation style (pulse vs snake) is set from the GUI and arrives in TLM_CORE.
 *   GPIO 0..3 are the four data lines (the freed stack-light MOSFET pins). */
#define LED_RING_COUNT          4
#define LED_RING_PINS           { 0, 1, 2, 3 }
#define LED_RING_LENS           { 120, 120, 30, 30 }
#define LED_RING_BRIGHTNESS     180u      /* 0..255 master cap (limits ring current) */
#define LED_RING_REFRESH_HZ     33u       /* render rate; show() blocks, so don't overdo it */

#define LED_ANIM_PULSE          0         /* must match firmware LED_MODE_PULSE */
#define LED_ANIM_SNAKE          1         /* must match firmware LED_MODE_SNAKE */

#define LED_TELEM_TIMEOUT_MS    2000u     /* no telemetry → DISCONNECT */
