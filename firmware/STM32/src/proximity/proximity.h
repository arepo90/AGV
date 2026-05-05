#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Proximity sensors (E18-D80NK on PC6/7/8/9 → EXTI lines 6/7/8/9).
 *
 *  EXTI is configured for both edges. The ISR re-reads the pin states and
 *  rebuilds the per-sensor bitmask each event, so we don't accumulate edge-
 *  counting drift. Whenever ANY sensor reads as obstructed, ESTOP_SRC_PROXIMITY
 *  is asserted; when ALL sensors read clear, the source is auto-cleared.
 *
 *  Pin polarity: E18-D80NK NPN open-collector pulls LOW on detection. With an
 *  internal pull-up the idle state is HIGH. PROX_ACTIVE_LOW from config.h
 *  selects the convention.
 *
 *  proximity_obstructed() returns the current per-sensor mask (bit per pin
 *  number, 6..9), useful for telemetry showing which side is blocked.
 * =============================================================================
 */

void     proximity_init(void);
uint8_t  proximity_obstructed(void);   /* bits 6..9 set if obstructed */

#endif /* PROXIMITY_H */
