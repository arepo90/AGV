#ifndef NAV_LINE_H
#define NAV_LINE_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  LINE_FOLLOW navigator (QTR-8A).
 *
 *  Per-frame min/max auto-ranging normalises each reading (no calibration); a
 *  weighted centroid gives line position in [-1, +1]; a PID maps position error
 *  → ω. Cruise speed drops on sharp corrections. This PID is the only one left
 *  in the firmware.
 *
 *  A wide perpendicular bar ("T", ≥ LINE_T_MIN_SENSORS black) at the end of the
 *  line triggers a 180° on-axis turn: rotate blind for LINE_TURN_BLIND_RAD of
 *  encoder-odometry heading, then keep rotating until the line is back in view,
 *  then resume following. Watchdogs (max sweep / timeout) fall back to lost.
 * =============================================================================
 */

void  nav_line_init(void);
void  nav_line_reset(void);
void  nav_line_get(float dt_s, float *v_target, float *omega_target);

/* Live tunables (PARAM_UPDATE). */
void  nav_line_set_cruise_mps(float v);
void  nav_line_set_lost_threshold(float t);
void  nav_line_set_t_black_counts(float counts);
void  nav_line_set_gains(float kp, float ki, float kd);
void  nav_line_set_t_min_sensors(float n);
void  nav_line_set_t_debounce_ticks(float n);
void  nav_line_set_reacquire_ticks(float n);
void  nav_line_set_turn_ccw(float v);              /* >= 0.5 → CCW, else CW */
void  nav_line_set_turn_omega_radps(float w);
void  nav_line_set_turn_blind_rad(float a);
void  nav_line_set_turn_max_rad(float a);
void  nav_line_set_turn_timeout_ms(float t);

/* Telemetry. */
bool  nav_line_is_lost(void);
float nav_line_position(void);      /* [-1, +1], computed line centroid */

#endif /* NAV_LINE_H */
