#include "encoders.h"
#include "config.h"
#include "qenc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* One revolution = ENCODER_COUNTS_PER_REV counts; arc per count at the rim. */
#define M_PER_COUNT ((2.0f * (float)M_PI * (float)WHEEL_RADIUS_M) \
                     / (float)ENCODER_COUNTS_PER_REV)

static int32_t  s_count[2];
static uint16_t s_last[2];
static float    s_vel_mps[2];

void encoders_init(void) {
    qenc_init();
    encoders_reset();
}

void encoders_reset(void) {
    s_count[SIDE_LEFT]   = 0;
    s_count[SIDE_RIGHT]  = 0;
    s_last[SIDE_LEFT]    = qenc_raw(0);
    s_last[SIDE_RIGHT]   = qenc_raw(1);
    s_vel_mps[SIDE_LEFT]  = 0.0f;
    s_vel_mps[SIDE_RIGHT] = 0.0f;
}

static void update_side(side_t side, uint8_t ch, bool invert, float dt_s) {
    uint16_t now = qenc_raw(ch);
    /* int16 cast handles wrap regardless of timer width. */
    int32_t d = (int16_t)(now - s_last[side]);
    s_last[side] = now;
    if (invert) d = -d;

    s_count[side] += d;

    float raw_mps = ((float)d * M_PER_COUNT) / dt_s;
    /* 1-pole low-pass: v = α·new + (1-α)·prev. */
    s_vel_mps[side] += ENCODER_VEL_LPF_ALPHA * (raw_mps - s_vel_mps[side]);
}

void encoders_tick(float dt_s) {
    if (dt_s <= 0.0f) return;
    update_side(SIDE_LEFT,  0, (ENCODER_INVERT_LEFT  != 0), dt_s);
    update_side(SIDE_RIGHT, 1, (ENCODER_INVERT_RIGHT != 0), dt_s);
}

int32_t encoders_count(side_t side)        { return s_count[side]; }
float   encoders_velocity_mps(side_t side) { return s_vel_mps[side]; }
