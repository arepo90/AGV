#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>
#include "types.h"

/* =============================================================================
 *  Wheel encoders over hal/qenc.  (app tier)
 *
 *  encoders_tick(dt) reads the hardware counters, accumulates signed distance,
 *  and estimates per-wheel linear velocity (m/s at the rim). The raw finite
 *  difference is quantisation-noisy at low speed, so a 1-pole low-pass
 *  (ENCODER_VEL_LPF_ALPHA) smooths it before the velocity PI consumes it.
 * =============================================================================
 */

void    encoders_init(void);
void    encoders_reset(void);
void    encoders_tick(float dt_s);

int32_t encoders_count(side_t side);          /* cumulative signed counts */
float   encoders_velocity_mps(side_t side);   /* filtered rim velocity */

#endif /* ENCODERS_H */
