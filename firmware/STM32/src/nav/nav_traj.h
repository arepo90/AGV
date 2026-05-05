#ifndef NAV_TRAJ_H
#define NAV_TRAJ_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  TRAJECTORY_FOLLOW navigator — point-and-steer over a waypoint list.
 *
 *  Algorithm (each tick):
 *    1. If active waypoint index >= count → trajectory complete, output (0,0).
 *    2. Compute (dx, dy) = waypoint - pose.
 *    3. If distance < WAYPOINT_REACH_RADIUS_M → mark reached, advance index.
 *    4. Otherwise:
 *         heading_error = wrap_pi(atan2(dy, dx) - theta)
 *         ω_target      = TRAJECTORY_HEADING_KP · heading_error
 *         v_target      = TRAJECTORY_CRUISE_MPS, halved when |heading_error|
 *                         exceeds TRAJECTORY_SLOWDOWN_RAD.
 *
 *  Pure pursuit could replace this in a future polish pass for smoother
 *  curves over densely-spaced waypoints. The simple formulation here is
 *  robust and works well for sparse waypoints (≥0.5 m apart).
 *
 *  Waypoint upload:
 *    nav_traj_clear() and nav_traj_add() are called from cmd.c when the
 *    workstation issues CMD_LOAD_TRAJECTORY. Buffer is RAM-only — power
 *    cycle clears the trajectory. Persistence isn't on the roadmap; the
 *    workstation owns the source of truth.
 * =============================================================================
 */

void  nav_traj_init(void);
void  nav_traj_get(float dt_s, float *v_target, float *omega_target);
void  nav_traj_reset(void);   /* PID reset only; waypoints preserved */

void  nav_traj_clear(void);
bool  nav_traj_add(float x, float y);   /* false if buffer full */
uint8_t nav_traj_count(void);
uint8_t nav_traj_active_index(void);
bool  nav_traj_complete(void);

#endif /* NAV_TRAJ_H */
