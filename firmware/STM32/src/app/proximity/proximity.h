#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <stdint.h>

/* =============================================================================
 *  E18-D80NK proximity sensors on PC6/7/8/9 → EXTI lines 6/7/8/9.  (app tier)
 *
 *  Both-edge EXTI. The ISR re-reads the pins and rebuilds the per-sensor mask
 *  each event (no edge-count drift). Any sensor obstructed asserts the
 *  proximity E-STOP source; all clear auto-clears it. This is the sole user of
 *  its GPIO/EXTI, so it owns those registers directly.
 *
 *  proximity_obstructed() returns the live mask remapped to the canonical
 *  FL=bit6, FR=bit7, RL=bit8, RR=bit9 telemetry layout, per the
 *  PROX_CORNER_* wiring map in config.h (logical corner != physical pin).
 * =============================================================================
 */

void     proximity_init(void);
uint16_t proximity_obstructed(void);

#endif /* PROXIMITY_H */
