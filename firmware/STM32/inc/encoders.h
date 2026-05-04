#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

/* =============================================================================
 *  Hardware quadrature decoders (TIM2 / TIM3 in encoder mode).
 *
 *  TIM2: PA0 (CH1) / PA1 (CH2) — LEFT wheel
 *  TIM3: PA6 (CH1) / PA7 (CH2) — RIGHT wheel
 *
 *  Both timers are configured for x4 quadrature decoding (count both edges of
 *  both channels). The hardware does the counting; this module just snapshots
 *  the registers periodically and computes deltas / wheel velocity.
 *
 *  TIM2 is 32-bit on the F0; TIM3 is 16-bit. We mask both reads to 16 bits and
 *  use a signed-cast wrap trick to get correct deltas across rollover, so the
 *  two timers behave identically from this module's perspective.
 *
 *  encoders_tick(dt_s) must be called once per control loop iteration. Reading
 *  velocity without ticking returns the previous tick's value.
 * =============================================================================
 */

typedef enum {
    ENC_LEFT  = 0,
    ENC_RIGHT = 1,
} encoder_side_t;

void     encoders_init(void);
void     encoders_tick(float dt_s);

int32_t  encoders_count(encoder_side_t side);     /* accumulated signed count */
float    encoders_velocity_mps(encoder_side_t side);  /* linear m/s at wheel */
float    encoders_velocity_rpm(encoder_side_t side);

void     encoders_reset(void);

#endif /* ENCODERS_H */
