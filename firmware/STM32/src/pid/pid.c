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
    p->integral = 0.0f;
    p->prev_meas = 0.0f;
    p->initialised = false;
}

float pid_step(pid_t *p, float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* Integral with anti-windup clamp. */
    p->integral += error * dt;
    if (p->integral >  p->i_limit) p->integral =  p->i_limit;
    if (p->integral < -p->i_limit) p->integral = -p->i_limit;

    /* Derivative on measurement (not on error) — prevents derivative kick when
     * setpoint steps. First call has no valid prev_meas; skip D term. */
    float d_meas = 0.0f;
    if (p->initialised) {
        d_meas = (measurement - p->prev_meas) / dt;
    }
    p->prev_meas = measurement;
    p->initialised = true;

    float u = p->kp * error + p->ki * p->integral - p->kd * d_meas;

    if (u > p->out_max) u = p->out_max;
    if (u < p->out_min) u = p->out_min;
    return u;
}
