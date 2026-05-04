#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Heartbeat watcher (only meaningful in SUPERVISED mode).
 *
 *  Two-stage degradation per architecture:
 *    Stage 0 — healthy           (last HB within HEARTBEAT_TIMEOUT_MS)
 *    Stage 1 — mode degraded     (timeout exceeded → switch to UNSUPERVISED)
 *    Stage 2 — virtual E-STOP    (timeout + grace exceeded)
 *
 *  Receiving a heartbeat at any stage:
 *    - Returns to stage 0
 *    - If we were in UNSUPERVISED *because* of the timeout, restore SUPERVISED
 *    - The HEARTBEAT_TIMEOUT virtual-E-STOP source (if asserted) is NOT
 *      auto-cleared; the workstation must explicitly clear it. This is
 *      intentional — connection flapping should not silently re-enable motors.
 *
 *  In UNSUPERVISED mode the watcher is dormant — no timeout tracked.
 * =============================================================================
 */

void heartbeat_init(void);
void heartbeat_received(void);   /* call from PKT_HEARTBEAT and PKT_CMD handlers */
void heartbeat_tick(void);       /* call once per main-loop iteration */

uint32_t heartbeat_ms_since_last(void);
uint8_t  heartbeat_stage(void);

#endif /* HEARTBEAT_H */
