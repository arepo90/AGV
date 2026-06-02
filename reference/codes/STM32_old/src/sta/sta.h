#ifndef STA_H
#define STA_H

/* =============================================================================
 *  Reusable super-twisting sliding-mode controller (Levant).
 *
 *  Sliding surface: s = setpoint - measurement  (pure error).
 *
 *      u̇₁ = k₂ · sign(s)
 *      u  = k₁ · √|s| · sign(s) + u₁
 *
 *  The integral state u₁ is clamped to ±u1_limit (anti-windup; symmetric so
 *  live gain changes don't blow up the state). The output u is clamped to
 *  [u_min, u_max].
 *
 *  Sign convention: positive plant gain assumed — a positive setpoint - meas
 *  error produces a positive u (drives meas up). For wheel velocity control
 *  this means positive duty → positive measured velocity; for chassis
 *  velocity loops the corrected setpoint feeding the kinematic split.
 *
 *  Discretisation: forward Euler on u₁, evaluated at the same dt as the rest
 *  of the cascade tick. The discontinuous sign() inside the integral is the
 *  textbook STA form; at the 200 Hz control loop the residual chattering is
 *  acceptable. If the actuator complains, swap sign() for tanh(s/ε) here.
 * =============================================================================
 */

typedef struct {
    float k1, k2;
    float u1;          /* integrator state */
    float u_min, u_max;
    float u1_limit;
} sta_t;

void  sta_init(sta_t *p, float k1, float k2,
               float u_min, float u_max, float u1_limit);
void  sta_set_gains(sta_t *p, float k1, float k2);
void  sta_reset(sta_t *p);
float sta_step(sta_t *p, float setpoint, float measurement, float dt);

#endif /* STA_H */
