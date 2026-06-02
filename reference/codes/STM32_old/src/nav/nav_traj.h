#ifndef NAV_TRAJ_H
#define NAV_TRAJ_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  TRAJECTORY_FOLLOW navigator — pure-pursuit over a waypoint polyline.
 *
 *  Path model: straight-line segments between consecutive waypoints. The
 *  "implicit waypoint 0" is the AGV's pose captured at the moment the
 *  trajectory starts; this anchors the first segment so cross-track error
 *  is well-defined from tick 0.
 *
 *  Per-tick algorithm:
 *    1. Project current pose onto the active segment. Advance the active
 *       index when the projection parameter t crosses 1.0 (passed the
 *       endpoint along the segment direction).
 *    2. Walk s_lookahead_m forward along the path from the projection,
 *       crossing segment boundaries as needed; clamp at the last waypoint.
 *       The endpoint of this walk is the lookahead point.
 *    3. α = wrap_pi(atan2(dy_lookahead, dx_lookahead) - θ)
 *       κ = 2·sin(α) / ||AGV - lookahead||      (curvature of the
 *                                                 connecting arc)
 *       v = TRAJECTORY_CRUISE_MPS / (1 + TRAJECTORY_CURV_SLOWDOWN·|κ|)
 *       ω = clamp(v · κ,  ±MAX_ANGULAR_SPEED_RADPS)
 *    4. If |α| > π/2 (lookahead is behind us), output v=0 and turn in
 *       place toward the lookahead — pure pursuit can't drive backward.
 *    5. Trajectory completes when within WAYPOINT_REACH_RADIUS_M of the
 *       last waypoint, OR when the lookahead walk runs past it and the
 *       projection clears it.
 *
 *  Properties this gives you (over the old point-and-steer):
 *    - Smooth arcs across waypoint corners (lookahead sees over the corner
 *      before the AGV arrives).
 *    - Returns to the path if pushed off (segment projection pulls the
 *      lookahead toward the line, not the next waypoint).
 *    - Curvature-aware speed: tight corners auto-slow so v·|κ| stays below
 *      ω_max, the cascade doesn't saturate, the AGV doesn't leave the path.
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
void  nav_traj_reset(void);   /* re-anchors first-segment start; waypoints preserved */

void  nav_traj_clear(void);
bool  nav_traj_add(float x, float y);   /* false if buffer full */
uint8_t nav_traj_count(void);
uint8_t nav_traj_active_index(void);
bool  nav_traj_complete(void);

/* Live-tunable parameters (PARAM_UPDATE entry points). */
void  nav_traj_set_cruise_mps(float v);
void  nav_traj_set_lookahead_m(float m);
void  nav_traj_set_curv_slowdown(float g);

#endif /* NAV_TRAJ_H */
