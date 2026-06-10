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
#define PROTO_VERSION           0x04u   /* v4: TOF + 6S battery removed (SENSORS = 18B fixed + lidar tail) */
#define PROTO_MAX_PAYLOAD       255u
#define PROTO_FRAME_OVERHEAD    8u
#define PROTO_MAX_FRAME         (PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD)
#define PKT_BATTERY             0x0Cu   /* ESP32→STM32: pushed 3S bus voltage (u16 mV LE); mirrors proto.h */

/* ---- Battery monitor (INA219, moved here from the STM32) ------------------ */
/* One INA219 senses the 3S pack voltage (shunt unused; A0=A1=GND). The ESP32
 * polls it and pushes each reading to the STM32 as PKT_BATTERY — the firmware
 * keeps the low-voltage caution/E-STOP decision and echoes the value in
 * TLM_SENSORS, so the GUI/panel path is unchanged. */
#define BATT_I2C_SDA_PIN        6
#define BATT_I2C_SCL_PIN        7
#define BATT_I2C_FREQ_HZ        100000u   /* one device, standard mode is plenty */
#define BATT_INA219_ADDR        0x40
#define BATT_POLL_HZ            2u        /* mirrors the STM32's BATTERY_POLL_HZ */
#define BATT_INJECT_FORCE_MS    100u      /* host stream quiet mid-frame this long → inject anyway */

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
#define LED_RING_LENS           { 107, 35, 35, 122 }
#define LED_RING_BRIGHTNESS     180u      /* 0..255 master cap (limits ring current) */
#define LED_RING_REFRESH_HZ     33u       /* render rate; show() blocks, so don't overdo it */

#define LED_ANIM_PULSE          0         /* must match firmware LED_MODE_PULSE */
#define LED_ANIM_SNAKE          1         /* must match firmware LED_MODE_SNAKE */

#define LED_TELEM_TIMEOUT_MS    2000u     /* no telemetry → DISCONNECT */

/* ---- Distance-reactive indicator ring -----------------------------------
 * One of the two big rings becomes a top-down obstacle display instead of the
 * shared state animation. Its base layer is operator-selectable (off / white,
 * via TLM_CORE led_indicator_cfg bit0), and each "indicator point" maps to a
 * sensor whose reading colours a span of LEDs centred on that point:
 *   - IR (E18-D80NK):  red on detection, fixed span.
 *   - LiDAR segment:   yellow→red gradient by distance, fixed span.
 * Overlapping spans resolve to the nearest (lowest-distance) reading. The other
 * three rings keep the state-colour pulse/snake animation.
 *
 *   >>> POPULATE THE WIRING-DEPENDENT VALUES BELOW. The placeholders build and
 *       run; the LED ids / ring index / spans are guesses until you fill them. */
#define LED_INDICATOR_RING        3         /* which ring index (0..LED_RING_COUNT-1) is reactive */

/* Indicator-point types. */
#define IND_TYPE_IR               1
#define IND_TYPE_LIDAR            2         /* used internally for the lidar arc; not in the table */

/* Sensor indices: IR = proximity_obstructed bit positions PC6..PC9 =
 * Front,Rear,Left,Right. */
#define IND_IR_FRONT              6
#define IND_IR_REAR               7
#define IND_IR_LEFT               8
#define IND_IR_RIGHT              9

/* Indicator-point table: { led_id, type, sensor_index }. led_id is the LED on
 * LED_INDICATOR_RING sitting at that sensor's physical location. */
#define LED_IND_POINTS  { \
    {  110, IND_TYPE_IR,  IND_IR_FRONT  }, \
    {  50, IND_TYPE_IR,  IND_IR_RIGHT  }, \
    {  10, IND_TYPE_IR,  IND_IR_REAR   }, \
    { 68, IND_TYPE_IR,  IND_IR_LEFT   }, \
}

/* LiDAR occupies an arc between two endpoint LEDs (the 0° and MAX_FOV° detections).
 * The N received segments map to evenly spaced bin centres along it (linear in LED
 * index, so pick endpoints on a span that doesn't cross the ring's 0 seam). */
#define LED_LIDAR_MAX_SEGMENTS    32u       /* must match firmware LIDAR_MAX_SEGMENTS */
#define LED_LIDAR_POINT_0         110       /* LED at lidar 0° */
#define LED_LIDAR_POINT_MAX       119       /* LED at lidar MAX_FOV° */

/* Gradient + visibility ranges (mm): full red at/below MIN, yellow at MAX, base
 * (no indication) beyond MAX. */
#define LED_LIDAR_MIN_MM          200
#define LED_LIDAR_MAX_MM          2000

/* Half-spread = LEDs lit on each side of a point (total span = 2·half + 1). */
#define LED_IR_HALFSPREAD         2
#define LED_LIDAR_HALFSPREAD      1

/* Indicator config bits in TLM_CORE byte 42 (mirror firmware LED_IND_*_BIT).
 * bit1 (spread mode) is currently unused — it shaped the removed TOF layer. */
#define LED_IND_BASE_BIT          0         /* 0 = base off, 1 = base white */
#define LED_BASE_WHITE_LEVEL      40u       /* white base brightness, pre master cap (0..255) */

/* Easing: fraction the rendered distance moves toward its target each frame, so
 * 5 Hz lidar telemetry animates smoothly at LED_RING_REFRESH_HZ. */
#define LED_IND_EASE_ALPHA        0.25f
