#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Motor current monitor.
 *
 *  Reads ADC current sense values and asserts ESTOP_SRC_OVERCURRENT if either
 *  motor exceeds MOTOR_OVERCURRENT_MA from config.h. The trip is sticky — the
 *  workstation must explicitly clear it (this is intentional: overcurrent
 *  usually means a stall, jam, or hardware fault that needs investigation).
 *
 *  De-glitching: the threshold must be exceeded for N consecutive ticks
 *  before tripping, to avoid nuisance trips from inrush and PWM ripple.
 * =============================================================================
 */

void current_monitor_init(void);
void current_monitor_tick(void);

#endif /* CURRENT_MONITOR_H */
