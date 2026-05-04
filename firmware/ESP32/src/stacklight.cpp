#include "stacklight.h"
#include "config.h"

#include <Arduino.h>
#include <math.h>

/* ---- LEDC channel allocation -------------------------------------------- */
#define CH_RED      0
#define CH_YELLOW   1
#define CH_GREEN    2
#define CH_BUZZER   3

#define DUTY_MAX    ((1u << STACK_LEDC_RESOLUTION) - 1u)

enum class StackState {
    INIT,        /* never seen telemetry — red breathing, no buzzer */
    DISCONNECT,  /* telemetry stopped — red breathing slow, buzzer slow */
    ESTOP,       /* estop_sources != 0 — red breathing fast, buzzer fast */
    CAUTION,     /* caution_sources != 0 — yellow solid */
    NORMAL,      /* all clear — green solid */
};

static StackState s_state = StackState::INIT;
static uint32_t   s_last_telem_ms = 0;
static bool       s_seen_telemetry = false;
static uint8_t    s_estop_sources = 0;
static uint8_t    s_caution_sources = 0;

/* ---- helpers ------------------------------------------------------------ */

static uint32_t breathing(uint32_t now_ms, float period_s) {
    /* Half-rectified cosine 0..1 (smooth pulse, never fully off feels nicer). */
    float t = (float)now_ms * 0.001f;
    float s = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * t / period_s);
    return (uint32_t)(s * (float)DUTY_MAX);
}

static bool square(uint32_t now_ms, uint32_t period_ms) {
    return ((now_ms / (period_ms / 2u)) & 1u) != 0u;
}

static StackState derive_state(uint32_t now_ms) {
    if (!s_seen_telemetry) return StackState::INIT;
    if ((now_ms - s_last_telem_ms) > STACK_TELEM_TIMEOUT_MS) return StackState::DISCONNECT;
    if (s_estop_sources   != 0) return StackState::ESTOP;
    if (s_caution_sources != 0) return StackState::CAUTION;
    return StackState::NORMAL;
}

/* ---- public API --------------------------------------------------------- */

void stacklight_init(void) {
    ledcSetup(CH_RED,    STACK_LEDC_FREQ_HZ, STACK_LEDC_RESOLUTION);
    ledcSetup(CH_YELLOW, STACK_LEDC_FREQ_HZ, STACK_LEDC_RESOLUTION);
    ledcSetup(CH_GREEN,  STACK_LEDC_FREQ_HZ, STACK_LEDC_RESOLUTION);
    ledcSetup(CH_BUZZER, STACK_LEDC_FREQ_HZ, STACK_LEDC_RESOLUTION);

    ledcAttachPin(STACK_PIN_RED,    CH_RED);
    ledcAttachPin(STACK_PIN_YELLOW, CH_YELLOW);
    ledcAttachPin(STACK_PIN_GREEN,  CH_GREEN);
    ledcAttachPin(STACK_PIN_BUZZER, CH_BUZZER);

    ledcWrite(CH_RED,    0);
    ledcWrite(CH_YELLOW, 0);
    ledcWrite(CH_GREEN,  0);
    ledcWrite(CH_BUZZER, 0);
}

void stacklight_update_from_telemetry(uint8_t estop_sources, uint8_t caution_sources,
                                      uint32_t now_ms) {
    s_estop_sources   = estop_sources;
    s_caution_sources = caution_sources;
    s_last_telem_ms   = now_ms;
    s_seen_telemetry  = true;
}

void stacklight_tick(uint32_t now_ms) {
    s_state = derive_state(now_ms);

    uint32_t red = 0, yellow = 0, green = 0, buzzer = 0;

    switch (s_state) {
    case StackState::NORMAL:
        green = DUTY_MAX;
        break;

    case StackState::CAUTION:
        yellow = DUTY_MAX;
        break;

    case StackState::ESTOP:
        red    = breathing(now_ms, 0.5f);          /* 2 Hz */
        buzzer = square(now_ms, 500u) ? DUTY_MAX : 0u; /* 2 Hz pulses */
        break;

    case StackState::DISCONNECT:
        red    = breathing(now_ms, 2.0f);          /* 0.5 Hz */
        buzzer = square(now_ms, 1000u) ? DUTY_MAX : 0u; /* 1 Hz pulses */
        break;

    case StackState::INIT:
        red    = breathing(now_ms, 2.0f);          /* slow, silent */
        break;
    }

    ledcWrite(CH_RED,    red);
    ledcWrite(CH_YELLOW, yellow);
    ledcWrite(CH_GREEN,  green);
    ledcWrite(CH_BUZZER, buzzer);
}
