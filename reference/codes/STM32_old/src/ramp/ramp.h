#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Motion profiling (chassis-level acceleration / deceleration ramps).
 *
 *  Slew limiter that sits between the navigator and the cascade controller.
 *  Reshapes the raw (v_cmd, ω_cmd) the navigator produces into a smoothed
 *  (v_ramped, ω_ramped) so the wheels never see a step change. Applies
 *  uniformly to REMOTE_CONTROL, LINE_FOLLOW, and TRAJECTORY_FOLLOW.
 *
 *  Profile shapes (same shape applied to both axes; magnitudes are per-axis):
 *
 *    LINEAR       : constant max accel. v(t) is a trapezoid on a step input.
 *                   One param per axis: max_accel.
 *    SCURVE       : jerk-limited. Acceleration itself ramps. Smoother on
 *                   cargo; two params per axis: max_accel, max_jerk.
 *    EXPONENTIAL  : 1st-order filter, v_now += α·(v_cmd - v_now). Cheap and
 *                   smooth but never quite reaches target. One param per
 *                   axis: time constant τ.
 *    CUSTOM       : user-supplied normalised curve f(s): [0,1]→[0,1] with
 *                   f(0)=0, f(1)=1, piecewise-linear over ≤RAMP_CURVE_POINTS_MAX
 *                   control points. Per-segment duration is derived from the
 *                   velocity step and max_accel so the curve's steepest part
 *                   tops out at max_accel.
 *
 *  The caution modifier scales max_accel (and max_jerk) at step time. This
 *  matches the architecture: effective_max_accel = configured × caution.
 *
 *  Reset on E-STOP and on navigation-function change (control.c calls
 *  ramp_reset() via reset_all_pids).
 * =============================================================================
 */

#define RAMP_CURVE_POINTS_MAX   8u

typedef enum {
    RAMP_SHAPE_LINEAR       = 0,
    RAMP_SHAPE_SCURVE       = 1,
    RAMP_SHAPE_EXPONENTIAL  = 2,
    RAMP_SHAPE_CUSTOM       = 3,
} ramp_shape_t;

void  ramp_init(void);
void  ramp_reset(void);

/* Step the slew limiter. dt_s is the control loop period.
 * v_cmd_raw / w_cmd_raw are the navigator's request; outputs are the ramped
 * values to feed downstream. Caution modifier is read internally. */
void  ramp_step(float dt_s,
                float v_cmd_raw,  float w_cmd_raw,
                float *v_out,     float *w_out);

/* Live tunables (PARAM_UPDATE plumbing). */
void  ramp_set_shape(ramp_shape_t s);
void  ramp_set_max_accel_lin(float a_mpss);
void  ramp_set_max_accel_ang(float a_radpss);
void  ramp_set_max_jerk_lin(float j_mpsss);
void  ramp_set_max_jerk_ang(float j_radpsss);
void  ramp_set_tau_lin(float tau_s);
void  ramp_set_tau_ang(float tau_s);

/* Custom curve upload (CMD_LOAD_RAMP_CURVE):
 *   begin    : open a new build buffer
 *   add(s,f) : append one point; must be monotonically increasing in s
 *   commit   : atomic swap into the active curve; returns false if invalid
 *              (needs at least 2 points and must span s=0 to s=1) */
void  ramp_curve_begin(void);
bool  ramp_curve_add_point(float s, float f);
bool  ramp_curve_commit(void);
void  ramp_curve_cancel(void);

/* Telemetry / introspection */
ramp_shape_t ramp_shape(void);

#endif /* RAMP_H */
