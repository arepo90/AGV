#ifndef CAUTION_H
#define CAUTION_H

#include <stdint.h>
#include "types.h"

/* =============================================================================
 *  Caution modifier — graceful degradation between full operation and E-STOP.
 *
 *  Each source independently records a level [0.0 .. 1.0]. The active modifier
 *  is the minimum across all sources (most conservative wins). Sources clear
 *  themselves to 1.0 when their condition resolves.
 *
 *  Workstation override: CAUTION_SRC_WORKSTATION_FORCED can pin the modifier
 *  to a specific value (typically 1.0 to bypass the system) — this also goes
 *  through the min(), so it can only be MORE permissive than other sources if
 *  this source is set higher. To genuinely override "be cautious", call
 *  caution_workstation_override(1.0) and let the OTHER sources clear naturally
 *  when their conditions resolve.
 *
 *  The modifier is read at the single PWM-write point in control.c so it is
 *  impossible to bypass by code path.
 * =============================================================================
 */

void     caution_init(void);
void     caution_set(caution_source_t src, float level);   /* level in [0..1] */
void     caution_clear(caution_source_t src);              /* equivalent to set(src, 1.0) */
float    caution_modifier(void);                           /* current effective scalar */
uint8_t  caution_active_sources(void);                     /* bitmask of non-1.0 sources */
void     caution_set_workstation_override(float level);    /* GUI explicitly sets override (can be higher than firmware min) */

#endif /* CAUTION_H */
