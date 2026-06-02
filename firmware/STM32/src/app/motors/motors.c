#include "motors.h"
#include "config.h"
#include "pwm.h"

static bool  s_enabled = false;
static float s_duty[2] = { 0.0f, 0.0f };

void motors_init(void) {
    pwm_init();
    s_enabled = false;
    s_duty[SIDE_LEFT]  = 0.0f;
    s_duty[SIDE_RIGHT] = 0.0f;
}

void motors_set_enabled(bool enabled) {
    pwm_set_enabled(enabled);
    s_enabled = enabled;
}

bool motors_enabled(void) { return s_enabled; }

void motors_set_signed(side_t side, float duty) {
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    s_duty[side] = duty;

    bool inverted = (side == SIDE_LEFT) ? (MOTOR_INVERT_LEFT != 0)
                                        : (MOTOR_INVERT_RIGHT != 0);
    float d = inverted ? -duty : duty;
    bool reverse = (d < 0.0f);
    if (reverse) d = -d;

    uint8_t ch = (side == SIDE_LEFT) ? 0u : 1u;
    pwm_set_dir(ch, reverse);
    pwm_set_duty(ch, d);
}

float motors_duty(side_t side) { return s_duty[side]; }
