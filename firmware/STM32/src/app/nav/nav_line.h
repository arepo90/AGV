#ifndef NAV_LINE_H
#define NAV_LINE_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  LINE_FOLLOW navigator (QTR-8A) + sweep calibration persisted to flash.
 *
 *  Per-sensor white/black baselines normalise each reading; a weighted centroid
 *  gives line position in [-1, +1]; a PID maps position error → ω. Cruise speed
 *  drops on sharp corrections. This PID is the only one left in the firmware.
 * =============================================================================
 */

void  nav_line_init(void);
void  nav_line_reset(void);
void  nav_line_get(float dt_s, float *v_target, float *omega_target);

/* Sweep calibration state machine (CMD_QTR_CALIBRATE). */
void  nav_line_cal_begin(void);
void  nav_line_cal_track(void);     /* sample min/max while a sweep is active */
void  nav_line_cal_cancel(void);
bool  nav_line_cal_active(void);
bool  nav_line_cal_save(void);              /* persist baselines to flash */
bool  nav_line_cal_reset_defaults(void);    /* revert + erase flash */
bool  nav_line_load_calibration_from_flash(void);

/* Live tunables (PARAM_UPDATE). */
void  nav_line_set_cruise_mps(float v);
void  nav_line_set_lost_threshold(float t);
void  nav_line_set_gains(float kp, float ki, float kd);

/* Telemetry. */
bool  nav_line_is_lost(void);
float nav_line_position(void);      /* [-1, +1], computed line centroid */

#endif /* NAV_LINE_H */
