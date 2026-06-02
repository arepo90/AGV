#ifndef TOF_H
#define TOF_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  VL53L0X time-of-flight ranging, four sensors behind a TCA9548A I2C mux. (app)
 *
 *  All four VL53L0X keep their shared default address (0x29); the mux exposes
 *  exactly one channel at a time so they never collide. The BNO055 and the two
 *  INA219 sit upstream of the mux on the same bus and are unaffected.
 *
 *  Sensors run in continuous back-to-back ranging. tof_tick() services one
 *  sensor per call (round-robin), reading its latest result only if ready, so a
 *  single pass costs at most a few short, non-waiting I2C transactions and never
 *  perturbs the 100 Hz control loop. The threshold→caution/E-STOP policy lives
 *  in safety.c (mirroring the cargo monitor); this module is the sensor only.
 * =============================================================================
 */

void     tof_init(void);
void     tof_tick(uint32_t now_ms);

bool     tof_present(uint32_t i);          /* sensor i (0..TOF_NUM_SENSORS-1) initialised */
uint16_t tof_distance_mm(uint32_t i);      /* last reading; TOF_VALID_MAX_MM if clear/absent */
uint16_t tof_min_distance_mm(void);        /* min over present sensors (for the safety monitor) */
bool     tof_any_present(void);

#endif /* TOF_H */
