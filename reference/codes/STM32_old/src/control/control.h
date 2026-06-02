#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* =============================================================================
 *  Cascade controller — the single owner of motor PWM writes.
 *
 *  Both inner (per-wheel velocity) and outer (chassis (v, ω)) loops are
 *  super-twisting sliding-mode controllers (STA). Sliding surface is pure
 *  error in each loop. See sta.h for the algorithm.
 *
 *  Every iteration of control_tick(dt_s):
 *
 *    1. Read navigator target  (v_cmd, ω_cmd)
 *    2. Apply caution-modified speed limits (clamps the *setpoint*, so loops
 *       never chase a target above effective_max, eliminating windup)
 *    3. (optional, USE_OUTER_CASCADE) Outer STA vs odometry → corrected (v, ω)
 *    4. Kinematic split → (v_left_target, v_right_target)
 *    5. Inner per-wheel STA vs encoder velocity → signed duty
 *    6. Apply caution modifier × duty
 *    7. Write to motors (PWM + DIR; SLEEP is owned by main.c)
 *
 *  E-STOP behaviour: if estop_active(), all STAs are reset and PWM is set to
 *  0. This prevents integral windup across the E-STOP window so motion
 *  resumes cleanly when the operator clears the fault.
 *
 *  Caller is responsible for ensuring encoders_tick() and odometry_tick() are
 *  called ONCE before control_tick(), with the same dt_s, every iteration.
 * =============================================================================
 */

void control_init(void);
void control_tick(float dt_s);

/* Live-tunable gain plumbing. cmd.c routes PARAM_UPDATE here. */
void control_set_inner_gains_left(float k1, float k2);
void control_set_inner_gains_right(float k1, float k2);
void control_set_outer_lin_gains(float k1, float k2);
void control_set_outer_ang_gains(float k1, float k2);

/* Telemetry accessors */
float control_v_target(void);
float control_omega_target(void);
float control_duty_left(void);
float control_duty_right(void);
float control_v_left_target(void);   /* post-ramp, post-outer, post-split */
float control_v_right_target(void);

#endif /* CONTROL_H */
