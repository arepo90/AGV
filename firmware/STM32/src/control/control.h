#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* =============================================================================
 *  Cascade controller — the single owner of motor PWM writes.
 *
 *  Every iteration of control_tick(dt_s):
 *
 *    1. Read navigator target  (v_cmd, ω_cmd)
 *    2. Apply caution-modified speed limits (clamps the *setpoint*, so PIDs
 *       never chase a target above effective_max, eliminating windup)
 *    3. (optional, USE_OUTER_CASCADE) Outer PI vs odometry → corrected (v, ω)
 *    4. Kinematic split → (v_left_target, v_right_target)
 *    5. Inner per-wheel PID vs encoder velocity → signed duty
 *    6. Apply caution modifier × duty
 *    7. Write to motors (PWM + DIR; SLEEP is owned by main.c)
 *
 *  E-STOP behaviour: if estop_active(), all PIDs are reset and PWM is set to
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
void control_set_inner_gains_left(float kp, float ki, float kd);
void control_set_inner_gains_right(float kp, float ki, float kd);
void control_set_outer_lin_gains(float kp, float ki, float kd);
void control_set_outer_ang_gains(float kp, float ki, float kd);

/* Telemetry accessors */
float control_v_target(void);
float control_omega_target(void);
float control_duty_left(void);
float control_duty_right(void);

#endif /* CONTROL_H */
