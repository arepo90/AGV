#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Motor PWM + DIR + SLEEP pins for the Pololu G2 dual driver.  (hal tier)
 *
 *  Channel 0 = LEFT  : PA8  PWM (TIM1_CH1, AF2)  PB0 DIR   PB1 SLEEP
 *  Channel 1 = RIGHT : PA11 PWM (TIM1_CH4, AF2)  PB2 DIR   PB3 SLEEP
 *
 *  Raw register access only: magnitude → CCR, a direction bit, and a shared
 *  enable. Sign handling and config-driven polarity live in app/motors.c.
 *
 *  SLEEP LOW = driver Hi-Z (boot default, safe). SLEEP HIGH = outputs active.
 * =============================================================================
 */

void pwm_init(void);
void pwm_set_duty(uint8_t ch, float mag01);   /* magnitude 0..1 → CCR (clamped) */
void pwm_set_dir(uint8_t ch, bool reverse);
void pwm_set_enabled(bool enabled);           /* both SLEEP pins together */

#endif /* PWM_H */
