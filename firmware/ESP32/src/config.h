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
#define PROTO_VERSION           0x03u   /* v3: +led_indicator_cfg in CORE, lidar tail in SENSORS */
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

/* ---- Distance-reactive indicator ring -----------------------------------
 * One of the two big rings becomes a top-down obstacle display instead of the
 * shared state animation. Its base layer is operator-selectable (off / white,
 * via TLM_CORE led_indicator_cfg bit0), and each "indicator point" maps to a
 * distance sensor whose reading colours a span of LEDs centred on that point:
 *   - IR (E18-D80NK):  red on detection, fixed span, both modes identical.
 *   - TOF (VL53L0X):   yellow→red gradient by distance; FIXED span, or in
 *                      RESPONSIVE mode (bit1) a span that grows as range falls.
 *   - LiDAR segment:   yellow→red gradient, fixed span; responsive N/A.
 * Overlapping spans resolve to the nearest (lowest-distance) reading. The other
 * three rings keep the state-colour pulse/snake animation.
 *
 *   >>> POPULATE THE WIRING-DEPENDENT VALUES BELOW. The placeholders build and
 *       run; the LED ids / ring index / spans are guesses until you fill them. */
#define LED_INDICATOR_RING        0         /* which ring index (0..LED_RING_COUNT-1) is reactive */

/* Indicator-point types. */
#define IND_TYPE_TOF              0
#define IND_TYPE_IR               1
#define IND_TYPE_LIDAR            2         /* used internally for the lidar arc; not in the table */

/* Sensor indices (match firmware ordering). TOF mux order = Front,Rear,Left,Right;
 * IR = proximity_obstructed bit positions PC6..PC9 = Front,Rear,Left,Right. */
#define IND_TOF_FRONT             0
#define IND_TOF_REAR              1
#define IND_TOF_LEFT              2
#define IND_TOF_RIGHT             3
#define IND_IR_FRONT              6
#define IND_IR_REAR               7
#define IND_IR_LEFT               8
#define IND_IR_RIGHT              9

/* Indicator-point table: { led_id, type, sensor_index }. led_id is the LED on
 * LED_INDICATOR_RING sitting at that sensor's physical location. */
#define LED_IND_POINTS  { \
    {   0, IND_TYPE_TOF, IND_TOF_FRONT }, \
    {  30, IND_TYPE_TOF, IND_TOF_RIGHT }, \
    {  60, IND_TYPE_TOF, IND_TOF_REAR  }, \
    {  90, IND_TYPE_TOF, IND_TOF_LEFT  }, \
    {  15, IND_TYPE_IR,  IND_IR_FRONT  }, \
    {  45, IND_TYPE_IR,  IND_IR_RIGHT  }, \
    {  75, IND_TYPE_IR,  IND_IR_REAR   }, \
    { 105, IND_TYPE_IR,  IND_IR_LEFT   }, \
}

/* LiDAR occupies an arc between two endpoint LEDs (the 0° and MAX_FOV° detections).
 * The N received segments map to evenly spaced bin centres along it (linear in LED
 * index, so pick endpoints on a span that doesn't cross the ring's 0 seam). */
#define LED_LIDAR_MAX_SEGMENTS    32u       /* must match firmware LIDAR_MAX_SEGMENTS */
#define LED_LIDAR_POINT_0         110       /* LED at lidar 0° */
#define LED_LIDAR_POINT_MAX       119       /* LED at lidar MAX_FOV° */

/* Gradient + visibility ranges (mm): full red at/below MIN, yellow at MAX, base
 * (no indication) beyond MAX. */
#define LED_TOF_MIN_MM            20
#define LED_TOF_MAX_MM            200
#define LED_LIDAR_MIN_MM          200
#define LED_LIDAR_MAX_MM          2000

/* Half-spread = LEDs lit on each side of a point (total span = 2·half + 1).
 * FIXED mode uses *_FIXED. RESPONSIVE mode (TOF only) ramps half-spread from FAR
 * (at MAX_MM) to NEAR (at MIN_MM) — e.g. 1↔7 ⇒ 3↔15 LEDs over 200↔20 mm. */
#define LED_TOF_HALFSPREAD_FIXED  1
#define LED_TOF_HALFSPREAD_FAR    1
#define LED_TOF_HALFSPREAD_NEAR   7
#define LED_IR_HALFSPREAD         2
#define LED_LIDAR_HALFSPREAD      1

/* Indicator config bits in TLM_CORE byte 42 (mirror firmware LED_IND_*_BIT). */
#define LED_IND_BASE_BIT          0         /* 0 = base off, 1 = base white */
#define LED_IND_MODE_BIT          1         /* 0 = fixed, 1 = responsive */
#define LED_BASE_WHITE_LEVEL      40u       /* white base brightness, pre master cap (0..255) */

/* Easing: fraction the rendered distance moves toward its target each frame, so
 * 5 Hz TOF/lidar telemetry animates smoothly at LED_RING_REFRESH_HZ. */
#define LED_IND_EASE_ALPHA        0.25f
