#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Motor driver interface (Pololu G2 24V14, dual channel).
 *
 *  This module currently handles only the SLEEP-pin safety surface; PWM, DIR,
 *  and encoder integration come in the drivetrain phase.
 *
 *  Pololu G2 SLEEP pin behaviour:
 *    - HIGH  → driver enabled (outputs active)
 *    - LOW   → driver in sleep mode (outputs Hi-Z, motor coasts)
 *  Boot-time state must be LOW so the driver is asleep before any other code
 *  has run. SLEEP is asserted (LOW) whenever any virtual E-STOP source is set.
 *
 *  Pin map (from architecture.md §STM32 Pin Mapping):
 *    PB1 = M1_SLEEP, PB3 = M2_SLEEP
 * =============================================================================
 */

void motors_init(void);                    /* configures SLEEP pins LOW first thing */
void motors_set_enabled(bool enabled);     /* drives both SLEEP pins together */
bool motors_enabled(void);

#endif /* MOTORS_H */
