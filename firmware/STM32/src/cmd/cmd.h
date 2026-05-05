#ifndef CMD_H
#define CMD_H

#include "comms.h"

/* =============================================================================
 *  Packet dispatcher.
 *
 *  Called from main loop for each packet popped from comms_recv(). Handles:
 *    PKT_HEARTBEAT       → heartbeat_received(), ACK
 *    PKT_CMD             → sub-type dispatch (set mode/function, vel cmd,
 *                          virtual E-STOP, override, log dump/clear, …)
 *    PKT_PARAM_UPDATE    → in next phase (placeholder ACKs for now)
 *    PKT_RESET           → soft reset OR clear-all-E-STOP per payload[0]
 *    PKT_FRAG            → trajectory upload (next phase, placeholder NACK)
 *    PKT_ACK / PKT_NACK  → diagnostic only — STM32 does not retry outbound
 *    PKT_LOG/TELEMETRY   → not legal from workstation, NACKed
 *
 *  Every CMD that successfully applies generates an ACK; every rejected one
 *  generates a NACK with an error code from comms.h.
 * =============================================================================
 */

void cmd_dispatch(const packet_t *pkt);

#endif /* CMD_H */
