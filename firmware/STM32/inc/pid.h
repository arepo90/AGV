#ifndef PID_H
#define PID_H

#include <stdbool.h>

/* =============================================================================
 *  Reusable PI(D) controller with integral clamp and output clamp.
 *
 *  Form (parallel):
 *     u = Kp·e + Ki·∫e dt + Kd·(de/dt)
 *
 *  Anti-windup: integral is clamped to ±i_limit BEFORE Ki multiplication, so
 *  switching gains at runtime doesn't blow up the integrator.
 *
 *  pid_step(p, setpoint, measurement, dt) returns the clamped output.
 * =============================================================================
 */

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_meas;       /* derivative-on-measurement, not on error: avoids derivative kick */
    float out_min, out_max;
    float i_limit;
    bool  initialised;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float out_min, float out_max, float i_limit);
void  pid_set_gains(pid_t *p, float kp, float ki, float kd);
void  pid_reset(pid_t *p);
float pid_step(pid_t *p, float setpoint, float measurement, float dt);

#endif /* PID_H */
