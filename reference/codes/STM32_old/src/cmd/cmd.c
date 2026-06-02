#include "cmd.h"
#include "caution.h"
#include "config.h"
#include "estop.h"
#include "heartbeat.h"
#include "hx711.h"
#include "log.h"
#include "nav.h"
#include "nav_line.h"
#include "nav_traj.h"
#include "odometry.h"
#include "params.h"
#include "ramp.h"
#include "state.h"
#include "system.h"
#include "types.h"
#include <string.h>

/* Any packet from the workstation is proof of life, not just HEARTBEAT. */
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
    if (pkt->len < 1u + 2u * sizeof(float)) {
        comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
        return;
    }
    float linear, angular;
    memcpy(&linear,  &pkt->payload[1],                    sizeof linear);
    memcpy(&angular, &pkt->payload[1 + sizeof(float)],    sizeof angular);
    nav_remote_set(linear, angular);
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
    caution_set_workstation_override(scalar);
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
    case CMD_START_TARE:
        if (state_function() != FUNC_STANDBY) {
            comms_send_nack(pkt->seq, NACK_ILLEGAL_TRANSITION);
        } else {
            hx711_start_tare();
            comms_send_ack(pkt->seq, 0);
        }
        break;

    case CMD_LOAD_TRAJECTORY: {
        /* Payload: [0]=sub, [1]=op
         *   op=0  → clear list (no further bytes)
         *   op=1  → append waypoint: [2..5]=float x, [6..9]=float y
         * Workstation issues op=0 once, then op=1 N times. PKT_FRAG is reserved
         * for bulk uploads larger than MAX_WAYPOINTS but is not yet implemented. */
        if (pkt->len < 2) { comms_send_nack(pkt->seq, NACK_BAD_LENGTH); break; }
        if (state_function() == FUNC_TRAJECTORY_FOLLOW) {
            comms_send_nack(pkt->seq, NACK_ILLEGAL_TRANSITION);
            break;
        }
        uint8_t op = pkt->payload[1];
        if (op == 0) {
            nav_traj_clear();
            comms_send_ack(pkt->seq, 0);
        } else if (op == 1) {
            if (pkt->len < 2u + 2u * sizeof(float)) {
                comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
                break;
            }
            float x, y;
            memcpy(&x, &pkt->payload[2],                 sizeof x);
            memcpy(&y, &pkt->payload[2 + sizeof(float)], sizeof y);
            if (nav_traj_add(x, y)) {
                if (nav_traj_count() == 1) {
                    log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_TRAJECTORY_LOADED, 0);
                }
                comms_send_ack(pkt->seq, 0);
            } else {
                comms_send_nack(pkt->seq, NACK_BUSY);   /* buffer full */
            }
        } else {
            comms_send_nack(pkt->seq, NACK_UNKNOWN_SUBTYPE);
        }
        break;
    }

    case CMD_QTR_CALIBRATE: {
        /* Payload: [sub][op]
         *   op=0 begin   op=1 save+persist   op=2 cancel   op=3 reset+erase */
        if (pkt->len < 2) { comms_send_nack(pkt->seq, NACK_BAD_LENGTH); break; }
        uint8_t op = pkt->payload[1];
        if (state_function() != FUNC_STANDBY) {
            comms_send_nack(pkt->seq, NACK_ILLEGAL_TRANSITION);
            break;
        }
        switch (op) {
        case 0:  nav_line_cal_begin();             comms_send_ack(pkt->seq, 0); break;
        case 1:
            if (nav_line_cal_save()) comms_send_ack(pkt->seq, 0);
            else                     comms_send_nack(pkt->seq, NACK_BUSY);
            break;
        case 2:  nav_line_cal_cancel();            comms_send_ack(pkt->seq, 0); break;
        case 3:
            if (nav_line_cal_reset_defaults()) comms_send_ack(pkt->seq, 0);
            else                               comms_send_nack(pkt->seq, NACK_BUSY);
            break;
        default: comms_send_nack(pkt->seq, NACK_UNKNOWN_SUBTYPE); break;
        }
        break;
    }

    case CMD_LOAD_RAMP_CURVE: {
        /* Payload: [0]=sub, [1]=op
         *   op=0  → begin (clear build buffer)
         *   op=1  → add point: [2..5]=float s, [6..9]=float f
         *   op=2  → commit (atomic swap into active curve)
         *   op=3  → cancel
         * Workstation issues op=0, op=1 × N, op=2. Curve is stored RAM-only;
         * resend on connect (same convention as PID gains). */
        if (pkt->len < 2) { comms_send_nack(pkt->seq, NACK_BAD_LENGTH); break; }
        uint8_t op = pkt->payload[1];
        if (op == 0) {
            ramp_curve_begin();
            comms_send_ack(pkt->seq, 0);
        } else if (op == 1) {
            if (pkt->len < 2u + 2u * sizeof(float)) {
                comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
                break;
            }
            float s, f;
            memcpy(&s, &pkt->payload[2],                 sizeof s);
            memcpy(&f, &pkt->payload[2 + sizeof(float)], sizeof f);
            if (ramp_curve_add_point(s, f)) comms_send_ack(pkt->seq, 0);
            else                            comms_send_nack(pkt->seq, NACK_BUSY);
        } else if (op == 2) {
            if (ramp_curve_commit()) comms_send_ack(pkt->seq, 0);
            else                     comms_send_nack(pkt->seq, NACK_BAD_LENGTH);
        } else if (op == 3) {
            ramp_curve_cancel();
            comms_send_ack(pkt->seq, 0);
        } else {
            comms_send_nack(pkt->seq, NACK_UNKNOWN_SUBTYPE);
        }
        break;
    }

    case CMD_RESET_ODOMETRY:
        if (state_function() != FUNC_STANDBY) {
            comms_send_nack(pkt->seq, NACK_ILLEGAL_TRANSITION);
        } else {
            odometry_reset();
            comms_send_ack(pkt->seq, 0);
        }
        break;

    case CMD_READ_SENSOR:
        /* Telemetry covers everything; on-demand reads are deferred until a
         * concrete use case demands them. */
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
        /* heartbeat_received() already ran in link_alive(); just ACK. */
        comms_send_ack(pkt->seq, 0);
        break;

    case PKT_CMD:
        handle_cmd(pkt);
        break;

    case PKT_PARAM_UPDATE: {
        /* Walks N×(u8 id + f32 value) tuples. NACK if any tuple's id was
         * unknown; ACK if all applied. (Failed tuples log individually.) */
        uint32_t failed = params_apply_payload(pkt->payload, pkt->len);
        if (failed == 0) comms_send_ack(pkt->seq, 0);
        else             comms_send_nack(pkt->seq, NACK_UNKNOWN_PARAM);
        break;
    }

    case PKT_RESET: {
        /* Payload byte 0: 0 = clear all E-STOP, 1 = soft-reset firmware.
         * Empty payload defaults to clear-all-E-STOP. */
        uint8_t mode = (pkt->len > 0) ? pkt->payload[0] : 0u;
        if (mode == 1u) {
            comms_send_ack(pkt->seq, 0);
            log_record(LOG_MOD_SYSTEM, LOG_SEV_WARN, LOG_CODE_SOFT_RESET, 1);
            /* Hold for ~20 ms so the ACK frame finishes the UART DMA before
             * the reset tears down the peripheral. Uses system_now_ms() so the
             * wait is predictable across optimisation levels. */
            uint32_t until = system_now_ms() + 20u;
            while ((int32_t)(until - system_now_ms()) > 0) { }
            system_soft_reset();
        } else {
            estop_clear_all();
            comms_send_ack(pkt->seq, 0);
        }
        break;
    }

    case PKT_FRAG:
        /* Fragmented uploads (e.g. large trajectories) — not yet implemented. */
        comms_send_nack(pkt->seq, NACK_BUSY);
        break;

    case PKT_ACK:
    case PKT_NACK:
        /* Outbound retries are not tracked (telemetry/log are fire-and-forget),
         * so ACK/NACK from the workstation is informational. Log NACKs with the
         * echoed seq + error code so they're visible in diagnostics. */
        if (pkt->type == PKT_NACK && pkt->len >= 2) {
            log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_REMOTE_NACK,
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
