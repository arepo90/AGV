#include "cmd.h"
#include "caution.h"
#include "config.h"
#include "estop.h"
#include "heartbeat.h"
#include "log.h"
#include "state.h"
#include "system.h"
#include "types.h"
#include <string.h>

/* Receiving any packet from the workstation counts as proof of life — even if
 * the packet is something other than HEARTBEAT, the link is clearly up. */
static void link_alive(void) {
    heartbeat_received();
}

/* ---- CMD sub-type handlers ---------------------------------------------- */

static void handle_set_mode(const packet_t *pkt) {
    if (pkt->len < 2) {
        comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
        return;
    }
    agv_mode_t m = (agv_mode_t)pkt->payload[1];
    if (state_set_mode(m)) {
        comms_send_ack(pkt->seq, 0);
    } else {
        comms_send_nack(pkt->seq, NACK_ILLEGAL_TRANSITION);
    }
}

static void handle_set_function(const packet_t *pkt) {
    if (pkt->len < 2) {
        comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
        return;
    }
    agv_function_t f = (agv_function_t)pkt->payload[1];
    if (state_set_function(f)) {
        comms_send_ack(pkt->seq, 0);
    } else {
        comms_send_nack(pkt->seq, NACK_ILLEGAL_TRANSITION);
    }
}

static void handle_vel_cmd(const packet_t *pkt) {
    /* float linear, float angular — consumed by nav_remote in next phase.
     * For now, ACK so the protocol is exercisable end-to-end. */
    if (pkt->len < 1u + 2u * sizeof(float)) {
        comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
        return;
    }
    /* TODO(drivetrain phase): nav_remote_set_target(linear, angular); */
    comms_send_ack(pkt->seq, 0);
}

static void handle_virtual_estop(const packet_t *pkt) {
    estop_assert(ESTOP_SRC_WORKSTATION);
    comms_send_ack(pkt->seq, 0);
}

static void handle_override_estop(const packet_t *pkt) {
    if (pkt->len < 2) { comms_send_nack(pkt->seq, NACK_BAD_LENGTH); return; }
    uint8_t mask = pkt->payload[1];
    estop_force_clear(mask);
    comms_send_ack(pkt->seq, 0);
}

static void handle_override_caution(const packet_t *pkt) {
    if (pkt->len < 1u + sizeof(float)) { comms_send_nack(pkt->seq, NACK_BAD_LENGTH); return; }
    float scalar;
    memcpy(&scalar, &pkt->payload[1], sizeof scalar);
    caution_set(CAUTION_SRC_WORKSTATION_FORCED, scalar);
    comms_send_ack(pkt->seq, 0);
}

static void handle_log_dump(const packet_t *pkt) {
    /* Logs are already streamed continuously by main.c. The dump-request just
     * confirms reception; the workstation should already have everything it
     * needs. ACK and move on. */
    comms_send_ack(pkt->seq, 0);
}

static void handle_log_clear(const packet_t *pkt) {
    log_clear();
    comms_send_ack(pkt->seq, 0);
}

static void handle_cmd(const packet_t *pkt) {
    if (pkt->len < 1) { comms_send_nack(pkt->seq, NACK_BAD_LENGTH); return; }
    uint8_t sub = pkt->payload[0];
    switch (sub) {
    case CMD_SET_MODE:               handle_set_mode(pkt);          break;
    case CMD_SET_FUNCTION:           handle_set_function(pkt);      break;
    case CMD_VEL_CMD:                handle_vel_cmd(pkt);           break;
    case CMD_VIRTUAL_ESTOP:          handle_virtual_estop(pkt);     break;
    case CMD_OVERRIDE_ESTOP_SOURCE:  handle_override_estop(pkt);    break;
    case CMD_OVERRIDE_CAUTION:       handle_override_caution(pkt);  break;
    case CMD_LOG_DUMP_REQUEST:       handle_log_dump(pkt);          break;
    case CMD_LOG_CLEAR:              handle_log_clear(pkt);         break;
    case CMD_READ_SENSOR:
    case CMD_LOAD_TRAJECTORY:
    case CMD_START_TARE:
    case CMD_QTR_CALIBRATE:
        /* Implemented in later phases — placeholder NACK so workstation knows. */
        log_record(LOG_MOD_COMMS, LOG_SEV_INFO, LOG_CODE_UNKNOWN_CMD_SUBTYPE, sub);
        comms_send_nack(pkt->seq, NACK_BUSY);
        break;
    default:
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UNKNOWN_CMD_SUBTYPE, sub);
        comms_send_nack(pkt->seq, NACK_UNKNOWN_SUBTYPE);
        break;
    }
}

/* ---- Top-level dispatch ------------------------------------------------- */

void cmd_dispatch(const packet_t *pkt) {
    link_alive();

    switch (pkt->type) {
    case PKT_HEARTBEAT:
        comms_send_ack(pkt->seq, 0);   /* heartbeat_received already happened in link_alive */
        break;

    case PKT_CMD:
        handle_cmd(pkt);
        break;

    case PKT_PARAM_UPDATE:
        /* Wired up in the runtime-tunables phase. ACK silently for now so the
         * workstation does not retry endlessly; the values aren't applied yet. */
        log_record(LOG_MOD_COMMS, LOG_SEV_INFO, LOG_CODE_PARAM_ID_UNKNOWN, pkt->len);
        comms_send_ack(pkt->seq, 0);
        break;

    case PKT_RESET: {
        /* Payload byte 0: 0 = clear all E-STOP, 1 = soft-reset firmware.
         * Empty payload defaults to clear-all-E-STOP. */
        uint8_t mode = (pkt->len > 0) ? pkt->payload[0] : 0u;
        if (mode == 1u) {
            comms_send_ack(pkt->seq, 0);
            log_record(LOG_MOD_SYSTEM, LOG_SEV_WARN, LOG_CODE_SOFT_RESET, 1);
            /* Brief pause so ACK actually leaves the wire before reset. */
            for (volatile uint32_t i = 0; i < 480000u; i++) { }
            system_soft_reset();
        } else {
            estop_clear_all();
            comms_send_ack(pkt->seq, 0);
        }
        break;
    }

    case PKT_FRAG:
        /* Fragmented trajectory upload — implemented in nav phase. */
        comms_send_nack(pkt->seq, NACK_BUSY);
        break;

    case PKT_ACK:
    case PKT_NACK:
        /* STM32 doesn't track outbound retries (telemetry/log are fire-and-
         * forget). Any received ACK/NACK from the workstation is informational;
         * we log unexpected NACKs so they're visible in diagnostics. */
        if (pkt->type == PKT_NACK && pkt->len >= 2) {
            log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_BAD_CRC,
                       ((uint32_t)pkt->payload[0] << 8) | pkt->payload[1]);
        }
        break;

    case PKT_TELEMETRY:
    case PKT_LOG:
        /* Not legal from workstation. */
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UNKNOWN_PACKET_TYPE, pkt->type);
        comms_send_nack(pkt->seq, NACK_UNKNOWN_TYPE);
        break;

    default:
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UNKNOWN_PACKET_TYPE, pkt->type);
        comms_send_nack(pkt->seq, NACK_UNKNOWN_TYPE);
        break;
    }
}
