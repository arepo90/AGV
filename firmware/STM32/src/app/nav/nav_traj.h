#ifndef NAV_TRAJ_H
#define NAV_TRAJ_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  TRAJECTORY_FOLLOW navigator — pure pursuit over a waypoint polyline.
 *
 *  Path = straight segments between waypoints, plus an implicit "waypoint 0"
 *  captured from the pose at start so cross-track error is defined from tick 0.
 *  Per tick: project pose onto the active segment, walk a lookahead distance
 *  forward, steer along the arc to that point, and scale speed down with
 *  curvature. Reverses are not driven — it rotates toward a lookahead behind it.
 * =============================================================================
 */

void    nav_traj_init(void);
void    nav_traj_reset(void);    /* re-anchor segment 0 to current pose */
void    nav_traj_clear(void);    /* drop all waypoints */
bool    nav_traj_add(float x, float y);
void    nav_traj_get(float dt_s, float *v_target, float *omega_target);

uint8_t nav_traj_count(void);
bool    nav_traj_complete(void);

/* Live tunables (PARAM_UPDATE). */
void    nav_traj_set_cruise_mps(float v);
void    nav_traj_set_lookahead_m(float m);
void    nav_traj_set_curv_slowdown(float g);

#endif /* NAV_TRAJ_H */
