#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Chassis-level motion profile (slew limiter).  (app tier)
 *
 *  Sits between the navigator and the kinematic split, reshaping the raw
 *  (v_cmd, ω_cmd) into a profile the wheels can track so the inner PI never
 *  chases a step. Same shape on both axes; magnitudes are per-axis.
 *
 *    LINEAR      constant max accel (one param: max_accel)
 *    SCURVE      jerk-limited; accel itself ramps (max_accel, max_jerk)
 *    EXPONENTIAL 1st-order filter v += α·(v_cmd-v); never quite reaches (τ)
 *    CUSTOM      operator curve f(s):[0,1]→[0,1], ≤8 points; segment duration
 *                scales so the curve's steepest part tops out at max_accel
 *
 *  max_accel/max_jerk are scaled by safety_caution_modifier() at step time, so
 *  a loaded AGV under CAUTION ramps gentler automatically.
 * =============================================================================
 */

#define RAMP_CURVE_POINTS_MAX   8u

typedef enum {
    RAMP_SHAPE_LINEAR      = 0,
    RAMP_SHAPE_SCURVE      = 1,
    RAMP_SHAPE_EXPONENTIAL = 2,
    RAMP_SHAPE_CUSTOM      = 3,
} ramp_shape_t;

void ramp_init(void);
void ramp_reset(void);   /* zero accel/segment state (E-STOP / function change) */
void ramp_step(float dt_s, float v_cmd, float w_cmd, float *v_out, float *w_out);

/* Live tunables (PARAM_UPDATE). */
void ramp_set_shape(ramp_shape_t s);
void ramp_set_max_accel_lin(float a);
void ramp_set_max_accel_ang(float a);
void ramp_set_max_jerk_lin(float j);
void ramp_set_max_jerk_ang(float j);
void ramp_set_tau_lin(float t);
void ramp_set_tau_ang(float t);
ramp_shape_t ramp_shape(void);

/* Custom-curve upload (CMD_LOAD_RAMP_CURVE). */
void ramp_curve_begin(void);
bool ramp_curve_add_point(float s, float f);   /* monotonic in s, f∈[0,1] */
bool ramp_curve_commit(void);                  /* false if invalid */
void ramp_curve_cancel(void);

#endif /* RAMP_H */
