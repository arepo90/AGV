#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max, float i_limit) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->i_limit = i_limit;
    pid_reset(p);
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd) {
    p->kp = kp; p->ki = ki; p->kd = kd;
}

void pid_reset(pid_t *p) {
    p->integral    = 0.0f;
    p->prev_meas   = 0.0f;
    p->initialised = false;
}

float pid_step(pid_t *p, float setpoint, float measurement, float feedforward, float dt) {
    if (dt <= 0.0f) {
        float u = feedforward;
        if (u > p->out_max) u = p->out_max;
        if (u < p->out_min) u = p->out_min;
        return u;
    }

    float error = setpoint - measurement;

    /* Derivative on measurement (no kick on setpoint steps). */
    float d_meas = 0.0f;
    if (p->initialised) d_meas = (measurement - p->prev_meas) / dt;
    p->prev_meas   = measurement;
    p->initialised = true;

    /* Tentative integral, then decide whether to commit it. */
    float i_trial = p->integral + error * dt;
    if (i_trial >  p->i_limit) i_trial =  p->i_limit;
    if (i_trial < -p->i_limit) i_trial = -p->i_limit;

    float u = feedforward + p->kp * error + p->ki * i_trial - p->kd * d_meas;

    /* Conditional integration: hold the integrator when saturated unless the
     * error is driving the output back inside the limits. */
    if (u > p->out_max) {
        if (error < 0.0f) p->integral = i_trial;
        u = p->out_max;
    } else if (u < p->out_min) {
        if (error > 0.0f) p->integral = i_trial;
        u = p->out_min;
    } else {
        p->integral = i_trial;
    }
    return u;
}
