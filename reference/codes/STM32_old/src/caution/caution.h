#ifndef CAUTION_H
#define CAUTION_H

#include <stdint.h>
#include "types.h"

/* =============================================================================
 *  Caution modifier — graceful degradation between full operation and E-STOP.
 *
 *  Each firmware source independently records a level in [0.0, 1.0]. While no
 *  workstation override is active, caution_modifier() returns the minimum
 *  across all firmware sources (most conservative wins). Sources clear
 *  themselves to 1.0 when their condition resolves.
 *
 *  Workstation override (caution_set_workstation_override): full authority.
 *  Once set, caution_modifier() returns the workstation value directly,
 *  bypassing the firmware min entirely. The workstation can be both more
 *  permissive (relaxing speed constraints firmware would impose) and more
 *  restrictive (forcing extra caution beyond firmware). See architecture.md.
 *
 *  CAUTION_SRC_WORKSTATION_FORCED is reported in caution_active_sources() as
 *  a synthetic bit when the override is active and below NORMAL. Do NOT call
 *  caution_set() with that enum value — it's reserved for the override path.
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
