#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include "types.h"

/* =============================================================================
 *  Motor abstraction over hal/pwm.  (app tier)
 *
 *  motors_set_signed(side, duty): duty ∈ [-1, +1]. Sign selects direction,
 *  magnitude becomes PWM. Per-side polarity is corrected from the config
 *  MOTOR_INVERT_* flags here so the rest of the firmware never thinks about it.
 *
 *  SLEEP policy is owned by main.c: enabled when NOT (estop || STANDBY).
 * =============================================================================
 */

void  motors_init(void);
void  motors_set_signed(side_t side, float duty);
void  motors_set_enabled(bool enabled);
bool  motors_enabled(void);
float motors_duty(side_t side);     /* last applied signed duty (telemetry) */

#endif /* MOTORS_H */
