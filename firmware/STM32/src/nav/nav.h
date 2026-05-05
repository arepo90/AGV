#ifndef NAV_H
#define NAV_H

#include <stdint.h>

/* =============================================================================
 *  Navigator dispatcher.
 *
 *  control_tick(dt) calls nav_get_target(dt, ...) once per iteration. The
 *  implementation switches on the current state_function() and forwards to
 *  the active navigator. Each navigator returns its desired
 *  (v_target, ω_target) chassis setpoint.
 *
 *  STANDBY navigator → (0, 0).
 *  REMOTE_CONTROL    → latest workstation command (nav_remote_set / _get).
 *  LINE_FOLLOW       → nav_line (QTR-8A weighted centroid + PID).
 *  TRAJECTORY_FOLLOW → nav_traj (point-and-steer over waypoint list).
 *
 *  nav_reset() is called from control.c on E-STOP to clear navigator-
 *  internal PID state — same reasoning as resetting the cascade PIDs:
 *  prevents windup across the fault window.
 * =============================================================================
 */

void nav_init(void);
void nav_get_target(float dt_s, float *v_target, float *omega_target);
void nav_reset(void);

/* REMOTE_CONTROL navigator — set by cmd.c on CMD_VEL_CMD. */
void nav_remote_set(float linear, float angular);
void nav_remote_get(float *linear, float *angular);

#endif /* NAV_H */
