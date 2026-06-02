#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/* =============================================================================
 *  Mode × function state machine.
 *
 *  Mode legality:
 *    SUPERVISED   ←→  UNSUPERVISED  (transitions allowed by heartbeat / WS)
 *
 *  Function legality (depends on mode):
 *    STANDBY            : always
 *    REMOTE_CONTROL     : SUPERVISED only
 *    LINE_FOLLOW        : both
 *    TRAJECTORY_FOLLOW  : both
 *
 *  Side effect on SUPERVISED→UNSUPERVISED: if current function is SUPERVISED-
 *  only (REMOTE_CONTROL), it is forced to STANDBY. The UNSUPERVISED+nav-active
 *  caution baseline is then applied / lifted automatically as functions change.
 * =============================================================================
 */

void          state_init(void);

agv_mode_t    state_mode(void);
agv_function_t state_function(void);

/* Returns true if the requested transition was applied. False if illegal — and
 * a LOG_CODE_ILLEGAL_TRANSITION entry is recorded with the requested IDs. */
bool          state_set_mode(agv_mode_t new_mode);
bool          state_set_function(agv_function_t new_func);

bool          function_is_supervised_only(agv_function_t f);
bool          function_is_navigating(agv_function_t f);   /* anything other than STANDBY */

#endif /* STATE_H */
