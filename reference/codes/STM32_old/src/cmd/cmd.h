#ifndef CMD_H
#define CMD_H

#include "comms.h"

/* =============================================================================
 *  Packet dispatcher.
 *
 *  Called from main loop for each packet popped from comms_recv():
 *    PKT_HEARTBEAT      ACK
 *    PKT_CMD            sub-type dispatch (set mode/function, vel cmd, virtual
 *                       E-STOP, overrides, log ops, QTR cal, tare, …)
 *    PKT_PARAM_UPDATE   walk tuples, ACK if all applied else NACK
 *    PKT_RESET          payload[0] = 0 → clear all E-STOP, 1 → soft reset
 *    PKT_FRAG           reserved for fragmented uploads (not yet implemented)
 *    PKT_ACK / PKT_NACK informational only — STM32 doesn't track outbound retries
 *    PKT_LOG / PKT_TELEMETRY   not legal from workstation; NACKed
 *
 *  Receiving any packet (not just HEARTBEAT) refreshes the heartbeat watcher.
 *  Every applied CMD generates an ACK; every rejected one generates a NACK
 *  with an error code from comms.h.
 * =============================================================================
 */

void cmd_dispatch(const packet_t *pkt);

#endif /* CMD_H */
