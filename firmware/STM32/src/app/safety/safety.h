#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/* =============================================================================
 *  Supervisory & safety arbitration.  (app tier)
 *
 *  One cohesive module owning everything that decides whether and how hard the
 *  AGV is allowed to move:
 *    - operating mode + navigation function state machine
 *    - virtual E-STOP source bitmask (assert / auto-clear / force-clear)
 *    - caution modifier (per-source minimum, with workstation override)
 *    - heartbeat watchdog (two-stage graceful degradation)
 *    - sensor-derived monitors (cargo weight/imbalance, motor overcurrent)
 *
 *  These were six tiny interdependent modules before; merging removes a web of
 *  cross-includes. E-STOP assert/clear are ISR-safe (proximity calls them).
 * =============================================================================
 */

void safety_init(void);

/* ---- Mode / function state machine ---------------------------------------- */
agv_mode_t     safety_mode(void);
agv_function_t safety_function(void);
bool safety_set_mode(agv_mode_t mode);          /* false on illegal value */
bool safety_set_function(agv_function_t func);  /* false on illegal value / mode */
bool safety_function_is_navigating(agv_function_t f);

/* ---- Virtual E-STOP -------------------------------------------------------- */
void     safety_estop_assert(estop_source_t src);              /* ISR-safe */
void     safety_estop_clear_autoclearing(estop_source_t src);  /* ISR-safe; auto-clear set only */
void     safety_estop_force_clear(uint16_t mask);              /* workstation override */
void     safety_estop_clear_all(void);
bool     safety_estop_active(void);
uint16_t safety_estop_sources(void);

/* ---- Caution modifier ------------------------------------------------------ */
void    safety_caution_set(caution_source_t src, float level);
void    safety_caution_clear(caution_source_t src);
void     safety_caution_set_ws_override(float level);
float    safety_caution_modifier(void);     /* [0,1] speed/accel scalar */
uint16_t safety_caution_sources(void);

/* ---- Heartbeat watchdog ---------------------------------------------------- */
void    safety_heartbeat_received(void);
void    safety_heartbeat_tick(void);       /* call each main loop */
uint8_t safety_heartbeat_stage(void);

/* ---- Sensor-derived monitors (call at the control rate) -------------------- */
void    safety_monitors_tick(void);        /* cargo + overcurrent + TOF + battery */
void    safety_set_weight_caution_kg(float kg);
void    safety_set_weight_estop_kg(float kg);
void    safety_set_imbalance_caution(float frac);
void    safety_set_imbalance_estop(float frac);

/* ---- TOF distance bands + 3S low-voltage thresholds (runtime-tunable) ----- */
void    safety_set_tof_caution_mm(float mm);
void    safety_set_tof_critical_mm(float mm);
void    safety_set_tof_estop_mm(float mm);
void    safety_set_battery_caution_mv(float mv);
void    safety_set_battery_estop_mv(float mv);

/* ---- LiDAR distance bands (mm, runtime-tunable; same policy as TOF) -------- */
void    safety_set_lidar_caution_mm(float mm);
void    safety_set_lidar_critical_mm(float mm);
void    safety_set_lidar_estop_mm(float mm);

#endif /* SAFETY_H */
