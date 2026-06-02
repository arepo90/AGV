#include "ledring.h"
#include "config.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <math.h>

/* ---- Strips ------------------------------------------------------------- */
static const uint8_t  s_pins[LED_RING_COUNT] = LED_RING_PINS;
static const uint16_t s_lens[LED_RING_COUNT] = LED_RING_LENS;
static Adafruit_NeoPixel s_rings[LED_RING_COUNT];   /* configured in ledring_init */

/* ---- Tapped state ------------------------------------------------------- */
enum class StackState { INIT, DISCONNECT, ESTOP, CAUTION, NORMAL };

static uint16_t s_estop_sources   = 0;
static uint16_t s_caution_sources = 0;
static uint8_t  s_mode            = LED_ANIM_PULSE;
static uint32_t s_last_telem_ms   = 0;
static bool     s_seen_telemetry  = false;
static uint32_t s_last_render_ms  = 0;

struct Rgb { uint8_t r, g, b; };

/* ---- Helpers ------------------------------------------------------------ */

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
    case StackState::ESTOP:      return 1.0f;
    case StackState::CAUTION:    return 2.0f;
    case StackState::NORMAL:     return 2.5f;
    default:                     return 3.0f;
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
    float tail  = (float)n * 0.25f;
    float d = head - (float)i;
    if (d < 0.0f) d += (float)n;          /* wrap: distance behind the head */
    return (d <= tail) ? (1.0f - d / tail) : 0.0f;
}

static uint8_t scale(uint8_t c, float level) {
    int v = (int)((float)c * level + 0.5f);
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
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
}

void ledring_update_from_telemetry(uint16_t estop_sources, uint16_t caution_sources,
                                   uint8_t led_mode, uint32_t now_ms) {
    s_estop_sources   = estop_sources;
    s_caution_sources = caution_sources;
    s_mode            = led_mode;
    s_last_telem_ms   = now_ms;
    s_seen_telemetry  = true;
}

void ledring_tick(uint32_t now_ms) {
    if ((now_ms - s_last_render_ms) < (1000u / LED_RING_REFRESH_HZ)) return;
    s_last_render_ms = now_ms;

    StackState st = derive_state(now_ms);
    Rgb col = state_color(st);
    bool snake = (s_mode == LED_ANIM_SNAKE);
    float pPeriod = pulse_period_s(st);
    float sPeriod = snake_period_s(st);

    /* Pulse intensity is uniform across a ring — compute it once. */
    float pulse = snake ? 0.0f : pulse_level(now_ms, pPeriod);

    for (uint8_t r = 0; r < LED_RING_COUNT; r++) {
        uint16_t n = s_lens[r];
        for (uint16_t i = 0; i < n; i++) {
            float lvl = snake ? snake_level(i, n, now_ms, sPeriod) : pulse;
            s_rings[r].setPixelColor(i, s_rings[r].Color(scale(col.r, lvl),
                                                         scale(col.g, lvl),
                                                         scale(col.b, lvl)));
        }
        s_rings[r].show();
    }
}
