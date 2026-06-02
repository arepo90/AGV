#ifndef ESTOP_H
#define ESTOP_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/* =============================================================================
 *  Virtual E-STOP arbitration.
 *
 *  Multiple independent sources may assert. SLEEP pin control is owned by
 *  main.c, which checks both E-STOP state and FUNC_STANDBY to decide whether
 *  motors should be enabled.
 *
 *  Sources fall into two classes:
 *    - Auto-clearing (proximity, cargo): clear themselves when their condition
 *      resolves. Workstation can ALSO force-clear them early.
 *    - Sticky (heartbeat, workstation, overcurrent, firmware fault): require
 *      explicit clear via OVERRIDE_ESTOP_SOURCE or the RESET packet.
 *
 *  Physical E-STOP (power cut to driver) is NOT modeled here — it is a
 *  hardware concern and cannot be overridden in software.
 * =============================================================================
 */

void           estop_init(void);

void           estop_assert(estop_source_t src);            /* set bit, log on transition */
void           estop_clear_autoclearing(estop_source_t src);/* set bit's value to 0 if auto-clear */
void           estop_force_clear(uint8_t source_mask);      /* workstation override of any source */
void           estop_clear_all(void);                       /* RESET packet handler */

bool           estop_active(void);
uint8_t        estop_sources(void);                         /* current bitmask */

#endif /* ESTOP_H */
