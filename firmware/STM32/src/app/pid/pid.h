#ifndef PID_H
#define PID_H

#include <stdbool.h>

/* =============================================================================
 *  Reusable PID controller with feedforward and conditional-integration
 *  anti-windup.
 *
 *      u = clamp( ff + Kp·e + Ki·∫e dt − Kd·(dmeas/dt) , [out_min, out_max] )
 *
 *  - Feedforward `ff` is supplied per step (the wheel loops pass Kff·v_target;
 *    the line follower passes 0).
 *  - Anti-windup: the integrator only accumulates when doing so would NOT push
 *    an already-saturated output deeper into saturation. A magnitude clamp
 *    (i_limit) is the backstop so runtime gain changes can't explode it.
 *  - Derivative is on the measurement, not the error, so a setpoint step does
 *    not kick the output. Pass Kd = 0 to disable (the wheel velocity loops do).
 * =============================================================================
 */

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_meas;
    float out_min, out_max;
    float i_limit;
    bool  initialised;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float out_min, float out_max, float i_limit);
void  pid_set_gains(pid_t *p, float kp, float ki, float kd);
void  pid_reset(pid_t *p);
float pid_step(pid_t *p, float setpoint, float measurement, float feedforward, float dt);

#endif /* PID_H */
