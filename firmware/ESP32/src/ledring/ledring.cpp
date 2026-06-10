#include "ledring.h"
#include "config.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <math.h>

/* ---- Strips ------------------------------------------------------------- */
static const uint8_t  s_pins[LED_RING_COUNT] = LED_RING_PINS;
static const uint16_t s_lens[LED_RING_COUNT] = LED_RING_LENS;
static Adafruit_NeoPixel s_rings[LED_RING_COUNT];   /* configured in ledring_init */

/* Big rings are 120 LEDs; the per-LED overlap buffers are sized to that. LEDs on
 * a longer-than-expected ring beyond this index simply stay at the base colour. */
static const uint16_t RING_MAXLEN = 120;

/* ---- Tapped state ------------------------------------------------------- */
enum class StackState { INIT, DISCONNECT, ESTOP, CAUTION, NORMAL };

static uint16_t s_estop_sources   = 0;
static uint16_t s_caution_sources = 0;
static uint8_t  s_mode            = LED_ANIM_PULSE;
static uint32_t s_last_telem_ms   = 0;
static bool     s_seen_telemetry  = false;
static uint32_t s_last_render_ms  = 0;

/* Sensor state for the reactive ring (from the TLM_CORE + TLM_SENSORS taps). */
static uint8_t  s_indicator_cfg = 0;
static uint16_t s_prox_bits     = 0;
static uint16_t s_lidar_mm[LED_LIDAR_MAX_SEGMENTS];
static uint8_t  s_lidar_n       = 0;

/* Eased (smoothed) distances so 5 Hz telemetry renders fluidly at the frame rate. */
static float    s_lidar_ease[LED_LIDAR_MAX_SEGMENTS];

struct Rgb { uint8_t r, g, b; };

/* Indicator-point table (wiring-dependent; see config.h). */
struct IndPoint { uint16_t led; uint8_t type; uint8_t sensor; };
static const IndPoint s_points[]  = LED_IND_POINTS;
static const uint8_t  s_num_points = (uint8_t)(sizeof(s_points) / sizeof(s_points[0]));

/* ---- State-colour helpers (unchanged, used by the three non-reactive rings) -- */

static StackState derive_state(uint32_t now_ms) {
    if (!s_seen_telemetry)                                       return StackState::INIT;
    if ((now_ms - s_last_telem_ms) > LED_TELEM_TIMEOUT_MS)       return StackState::DISCONNECT;
    if (s_estop_sources   != 0)                                  return StackState::ESTOP;
    if (s_caution_sources != 0)                                  return StackState::CAUTION;
    return StackState::NORMAL;
}

static Rgb state_color(StackState s) {
    switch (s) {
    case StackState::NORMAL:  return { 0,   255, 0   };   /* green  */
    case StackState::CAUTION: return { 255, 160, 0   };   /* amber  */
    default:                  return { 255, 0,   0   };   /* red — ESTOP/INIT/DISCONNECT */
    }
}

/* Pulse period (s): faster = more urgent. */
static float pulse_period_s(StackState s) {
    switch (s) {
    case StackState::ESTOP:      return 0.6f;
    case StackState::CAUTION:    return 1.6f;
    case StackState::NORMAL:     return 2.5f;
    default:                     return 2.0f;   /* INIT / DISCONNECT — slow */
    }
}

/* Snake revolution period (s): time for the comet to circle the ring once. */
static float snake_period_s(StackState s) {
    switch (s) {
    case StackState::ESTOP:      return 0.5f;
    case StackState::CAUTION:    return 0.5f;
    case StackState::NORMAL:     return 0.5f;
    default:                     return 1.0f;
    }
}

/* Half-rectified cosine in [0,1] — never fully off, which reads as a soft breath. */
static float pulse_level(uint32_t now_ms, float period_s) {
    float t = (float)now_ms * 0.001f;
    return 0.5f - 0.5f * cosf(2.0f * (float)M_PI * t / period_s);
}

/* Comet intensity for LED i on an N-LED ring: bright at the head, fading over a
 * quarter-ring tail behind it. Head position is a normalised phase × N, so all
 * rings show the comet at the same fraction around the loop. */
static float snake_level(uint16_t i, uint16_t n, uint32_t now_ms, float period_s) {
    float phase = fmodf((float)now_ms * 0.001f / period_s, 1.0f);   /* [0,1) */
    float head  = phase * (float)n;
    float tail  = (float)n * 0.8f;
    float d = head - (float)i;
    if (d < 0.0f) d += (float)n;          /* wrap: distance behind the head */
    return (d <= tail) ? (1.0f - d / tail) : 0.0f;
}

static uint8_t scale(uint8_t c, float level) {
    int v = (int)((float)c * level + 0.5f);
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

/* ---- Reactive-ring helpers ---------------------------------------------- */

/* Yellow→red gradient: full red at/below min_mm, yellow at max_mm. */
static Rgb grad_color(float d, float min_mm, float max_mm) {
    float t = (d - min_mm) / (max_mm - min_mm);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return { 255, (uint8_t)(255.0f * t + 0.5f), 0 };
}

/* Paint a span centred on `center` (±half, wrapping) into the per-LED winners,
 * keeping only the nearest (lowest-distance) contribution at each LED. */
static void splat(uint16_t *best, Rgb *col, uint16_t n,
                  uint16_t center, int half, uint16_t dist, Rgb c) {
    for (int o = -half; o <= half; o++) {
        int idx = ((int)center + o) % (int)n;
        if (idx < 0) idx += (int)n;
        if (dist < best[idx]) { best[idx] = dist; col[idx] = c; }
    }
}

/* LED at the centre of LiDAR segment k of nseg, evenly spaced along the arc from
 * LED_LIDAR_POINT_0 to LED_LIDAR_POINT_MAX (linear in index — pick endpoints on a
 * span that does not cross the ring's 0 seam). */
static uint16_t lidar_center_led(uint8_t k, uint8_t nseg, uint16_t n) {
    float f = (nseg > 0) ? ((float)k + 0.5f) / (float)nseg : 0.5f;
    float pos = (float)LED_LIDAR_POINT_0 +
                ((float)LED_LIDAR_POINT_MAX - (float)LED_LIDAR_POINT_0) * f;
    int idx = (int)(pos + 0.5f) % (int)n;
    if (idx < 0) idx += (int)n;
    return (uint16_t)idx;
}

/* Step every eased distance toward its latest target. Segments beyond the current
 * count decay toward "clear" so a shrinking arc fades rather than snapping off. */
static void ease_step(void) {
    for (int k = 0; k < (int)LED_LIDAR_MAX_SEGMENTS; k++) {
        float tgt = (k < s_lidar_n) ? (float)s_lidar_mm[k] : (float)LED_LIDAR_MAX_MM;
        s_lidar_ease[k] += LED_IND_EASE_ALPHA * (tgt - s_lidar_ease[k]);
    }
}

static void render_indicator_ring(uint8_t r) {
    uint16_t n = s_lens[r];
    if (n > RING_MAXLEN) n = RING_MAXLEN;

    const bool base_white = (s_indicator_cfg >> LED_IND_BASE_BIT) & 1u;
    const Rgb  base = base_white
        ? Rgb{ (uint8_t)LED_BASE_WHITE_LEVEL, (uint8_t)LED_BASE_WHITE_LEVEL, (uint8_t)LED_BASE_WHITE_LEVEL }
        : Rgb{ 0, 0, 0 };

    static uint16_t best[RING_MAXLEN];
    static Rgb      best_col[RING_MAXLEN];
    for (uint16_t i = 0; i < n; i++) { best[i] = 0xFFFF; best_col[i] = base; }

    /* Fixed indicator points (IR). */
    for (uint8_t p = 0; p < s_num_points; p++) {
        const IndPoint pt = s_points[p];
        if (pt.led >= n) continue;
        if (pt.type != IND_TYPE_IR) continue;
        if (!((s_prox_bits >> pt.sensor) & 1u)) continue;         /* no detection → base */
        splat(best, best_col, n, pt.led, LED_IR_HALFSPREAD, 0u, Rgb{ 255, 0, 0 });
    }

    /* LiDAR arc: each fresh segment is a fixed-span gradient point. */
    for (uint8_t k = 0; k < s_lidar_n; k++) {
        float d = s_lidar_ease[k];
        if (d >= (float)LED_LIDAR_MAX_MM) continue;
        uint16_t dist = (uint16_t)(d < 0.0f ? 0.0f : (d > 65535.0f ? 65535.0f : d));
        splat(best, best_col, n, lidar_center_led(k, s_lidar_n, n),
              LED_LIDAR_HALFSPREAD, dist,
              grad_color(d, (float)LED_LIDAR_MIN_MM, (float)LED_LIDAR_MAX_MM));
    }

    for (uint16_t i = 0; i < n; i++)
        s_rings[r].setPixelColor(i, s_rings[r].Color(best_col[i].r, best_col[i].g, best_col[i].b));
    s_rings[r].show();
}

static void render_state_ring(uint8_t r, uint32_t now_ms,
                              Rgb col, bool snake, float pulse, float sPeriod) {
    uint16_t n = s_lens[r];
    for (uint16_t i = 0; i < n; i++) {
        float lvl = snake ? snake_level(i, n, now_ms, sPeriod) : pulse;
        s_rings[r].setPixelColor(i, s_rings[r].Color(scale(col.r, lvl),
                                                     scale(col.g, lvl),
                                                     scale(col.b, lvl)));
    }
    s_rings[r].show();
}

/* ---- API ---------------------------------------------------------------- */

void ledring_init(void) {
    for (uint8_t r = 0; r < LED_RING_COUNT; r++) {
        s_rings[r].updateType(NEO_GRB + NEO_KHZ800);
        s_rings[r].updateLength(s_lens[r]);
        s_rings[r].setPin(s_pins[r]);
        s_rings[r].begin();
        s_rings[r].setBrightness(LED_RING_BRIGHTNESS);
        s_rings[r].clear();
        s_rings[r].show();
    }
    for (int k = 0; k < (int)LED_LIDAR_MAX_SEGMENTS; k++) s_lidar_ease[k] = (float)LED_LIDAR_MAX_MM;
}

void ledring_update_from_telemetry(uint16_t estop_sources, uint16_t caution_sources,
                                   uint8_t led_mode, uint8_t indicator_cfg,
                                   uint16_t proximity_bits, uint32_t now_ms) {
    s_estop_sources   = estop_sources;
    s_caution_sources = caution_sources;
    s_mode            = led_mode;
    s_indicator_cfg   = indicator_cfg;
    s_prox_bits       = proximity_bits;
    s_last_telem_ms   = now_ms;
    s_seen_telemetry  = true;
}

void ledring_update_sensors(const uint16_t *lidar_mm, uint8_t lidar_n) {
    if (lidar_n > LED_LIDAR_MAX_SEGMENTS) lidar_n = LED_LIDAR_MAX_SEGMENTS;
    for (uint8_t k = 0; k < lidar_n; k++) s_lidar_mm[k] = lidar_mm[k];
    s_lidar_n = lidar_n;
}

void ledring_tick(uint32_t now_ms) {
    if ((now_ms - s_last_render_ms) < (1000u / LED_RING_REFRESH_HZ)) return;
    s_last_render_ms = now_ms;

    ease_step();

    StackState st = derive_state(now_ms);
    Rgb col = state_color(st);
    bool snake = (s_mode == LED_ANIM_SNAKE);
    float pulse   = snake ? 0.0f : pulse_level(now_ms, pulse_period_s(st));
    float sPeriod = snake_period_s(st);

    for (uint8_t r = 0; r < LED_RING_COUNT; r++) {
        if (LED_INDICATOR_RING < LED_RING_COUNT && r == (uint8_t)LED_INDICATOR_RING)
            render_indicator_ring(r);
        else
            render_state_ring(r, now_ms, col, snake, pulse, sPeriod);
    }
}
