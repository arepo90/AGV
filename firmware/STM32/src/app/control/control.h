#ifndef CONTROL_H
#define CONTROL_H

/* =============================================================================
 *  Per-tick control chain.  (app tier)
 *
 *    navigator → ramp → caution clamp → algebraic differential-drive split
 *      → per-wheel PI + feedforward → motors
 *
 *  No outer chassis cascade: wheel targets come straight from the kinematic
 *  split (robot geometry is assumed correct). Each wheel has an independent PI
 *  with velocity feedforward. On E-STOP the controllers and ramp are reset and
 *  PWM is zeroed (main.c also asserts SLEEP).
 * =============================================================================
 */

void  control_init(void);
void  control_tick(float dt_s);

/* Live gains (PARAM_UPDATE). */
void  control_set_pi_left(float kp, float ki);
void  control_set_pi_right(float kp, float ki);
void  control_set_kff_left(float kff);
void  control_set_kff_right(float kff);

/* Telemetry. */
float control_v_target(void);
float control_omega_target(void);
float control_v_left_target(void);
float control_v_right_target(void);
float control_duty_left(void);
float control_duty_right(void);

#endif /* CONTROL_H */
