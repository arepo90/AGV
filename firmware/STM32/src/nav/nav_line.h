#ifndef NAV_LINE_H
#define NAV_LINE_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  LINE_FOLLOW navigator — QTR-8A weighted-centroid line tracker.
 *
 *  Each tick:
 *    1. Read 8 QTR-8A raw values from the ADC scan buffer.
 *    2. Per-sensor: normalise raw → [0..1] using calibration (white/black
 *       baselines). Black line = high normalised value.
 *    3. Weighted centroid over sensor index. Position is mapped to [-1, +1]
 *       where 0 = line under array centre, +1 = line at right edge.
 *    4. PID(setpoint=0, measurement=position) → ω_target.
 *    5. v_target = LINE_FOLLOW_CRUISE_MPS, halved during a steep turn.
 *
 *  Line lost: if total normalised activity falls below
 *  QTR_LINE_LOST_THRESHOLD, output (0, 0) and log once. The robot stops in
 *  place until the line reappears or the function changes.
 * =============================================================================
 */

void nav_line_init(void);
void nav_line_get(float dt_s, float *v_target, float *omega_target);
void nav_line_reset(void);

bool  nav_line_is_lost(void);
float nav_line_position(void);   /* [-1, +1], last computed centroid */

/* Calibration setters (used by PARAM_UPDATE-style live tuning if you want
 * to set baselines manually; CMD_QTR_CALIBRATE is the normal path). */
void nav_line_set_calibration(uint32_t sensor_idx, uint16_t white, uint16_t black);

/* ---- Sweep-style calibration --------------------------------------------
 *
 *  Workstation flow:
 *    1. send CMD_QTR_CALIBRATE [op=0]   → nav_line_cal_begin()
 *    2. operator sweeps array left↔right over the line a few times
 *    3. send CMD_QTR_CALIBRATE [op=1]   → nav_line_cal_save()  (persists)
 *      OR send CMD_QTR_CALIBRATE [op=2] → nav_line_cal_cancel() (discard)
 *      OR send CMD_QTR_CALIBRATE [op=3] → nav_line_cal_reset_defaults()
 *
 *  While calibration is active, nav_line_cal_track() should be called every
 *  iteration that the ADC has fresh data; it updates per-sensor min/max from
 *  the live readings. The robot must be in STANDBY (cmd.c enforces).
 */
void nav_line_cal_begin(void);
void nav_line_cal_track(void);
bool nav_line_cal_save(void);                 /* true if persisted to flash */
void nav_line_cal_cancel(void);
bool nav_line_cal_reset_defaults(void);       /* erase flash, restore defaults */
bool nav_line_cal_active(void);

/* Loaded from flash on boot (called from main.c). Returns true if a valid
 * calibration record was found and applied. */
bool nav_line_load_calibration_from_flash(void);

/* Live-tunable parameters (PARAM_UPDATE entry points). */
void nav_line_set_cruise_mps(float v);
void nav_line_set_lost_threshold(float t);

#endif /* NAV_LINE_H */
