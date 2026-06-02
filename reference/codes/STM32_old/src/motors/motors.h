#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Pololu G2 24V14 dual motor driver.
 *
 *  Pin map (from architecture.md §STM32 Pin Mapping):
 *    Motor 1 (LEFT):  PA8  PWM (TIM1_CH1, AF2)
 *                     PB0  DIR
 *                     PB1  SLEEP
 *                     PA2  current sense  (handled in adc.c — future phase)
 *    Motor 2 (RIGHT): PA11 PWM (TIM1_CH4, AF2)
 *                     PB2  DIR
 *                     PB3  SLEEP
 *                     PA3  current sense
 *
 *  motors_set_pwm_signed(side, duty):
 *     duty ∈ [-1.0, +1.0]. Sign drives DIR; magnitude → CCR.
 *     The caller is responsible for the caution-modifier multiplication; this
 *     module just maps the signed duty into hardware registers.
 *
 *  SLEEP behaviour:
 *     LOW  → driver outputs Hi-Z. Boot default.
 *     HIGH → driver active (any motion function other than STANDBY, no E-STOP).
 *  main.c owns the sleep decision: sleep when estop_active() || FUNC_STANDBY.
 *  PWM brake is used to stop within a function; SLEEP is for true idle/fault.
 * =============================================================================
 */

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
} motor_side_t;

void motors_init(void);

/* SLEEP-pin control (driven by estop_apply()). */
void motors_set_enabled(bool enabled);
bool motors_enabled(void);

/* PWM + DIR write. duty is clamped to [-1, +1] internally. */
void motors_set_pwm_signed(motor_side_t side, float duty);

/* Last-applied values (for telemetry). */
float motors_last_duty(motor_side_t side);

#endif /* MOTORS_H */
