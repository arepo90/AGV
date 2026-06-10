#include "proto.h"
#include "battery.h"
#include "control.h"
#include "crc.h"
#include "lidar.h"
#include "loadcells.h"
#include "log.h"
#include "mcu.h"
#include "nav.h"
#include "nav_line.h"
#include "odometry.h"
#include "ramp.h"
#include "safety.h"
#include "telemetry.h"
#include "types.h"
#include "uart.h"
#include <string.h>

/* ===========================================================================
 *  Outbound
 * =========================================================================== */

static uint8_t s_tx_seq = 0;

bool proto_send(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t frame[PROTO_MAX_FRAME];
    frame[0] = PROTO_MAGIC0;
    frame[1] = PROTO_MAGIC1;
    frame[2] = PROTO_VERSION;
    frame[3] = s_tx_seq++;
    frame[4] = type;
    frame[5] = len;
    if (len && payload) memcpy(&frame[6], payload, len);

    uint16_t crc = crc16_ccitt(&frame[2], 4u + (uint32_t)len);   /* VER..end of payload */
    frame[6 + len]     = (uint8_t)(crc & 0xFFu);
    frame[6 + len + 1] = (uint8_t)(crc >> 8);
    return uart_send(frame, (uint16_t)(8u + len));
}

static bool send_ack(uint8_t seq, uint8_t status) {
    uint8_t p[2] = { seq, status };
    return proto_send(PKT_ACK, p, sizeof p);
}

static bool send_nack(uint8_t seq, uint8_t err) {
    uint8_t p[2] = { seq, err };
    return proto_send(PKT_NACK, p, sizeof p);
}

/* ===========================================================================
 *  Parameter application
 * =========================================================================== */

/* Mirror the PI pairs so a partial batch (just Kp, or just Ki) still pushes a
 * consistent pair to the controller. */
static float s_lkp = WHEEL_KP_LEFT,  s_lki = WHEEL_KI_LEFT;
static float s_rkp = WHEEL_KP_RIGHT, s_rki = WHEEL_KI_RIGHT;
static float s_line_kp = LINE_FOLLOW_KP, s_line_ki = LINE_FOLLOW_KI, s_line_kd = LINE_FOLLOW_KD;

/* Mirror the indicator-ring config byte so the two GUI toggles (base, spread mode)
 * each flip their own bit while leaving the other untouched. */
static uint8_t s_indicator_cfg = LED_INDICATOR_CFG_DEFAULT;

static bool param_apply_one(uint8_t id, float v) {
    /* HX711 corner-indexed params: corner = id - base. */
    if (id >= PARAM_HX711_OFFSET_BASE && id < PARAM_HX711_OFFSET_BASE + HX711_NUM_CORNERS) {
        loadcells_set_offset(id - PARAM_HX711_OFFSET_BASE, (int32_t)v);
        return true;
    }
    if (id >= PARAM_HX711_SCALE_BASE && id < PARAM_HX711_SCALE_BASE + HX711_NUM_CORNERS) {
        loadcells_set_scale(id - PARAM_HX711_SCALE_BASE, v);
        return true;
    }

    switch (id) {
    case PARAM_LEFT_KP:  s_lkp = v; control_set_pi_left(s_lkp, s_lki);  break;
    case PARAM_LEFT_KI:  s_lki = v; control_set_pi_left(s_lkp, s_lki);  break;
    case PARAM_LEFT_KFF: control_set_kff_left(v);                       break;
    case PARAM_RIGHT_KP: s_rkp = v; control_set_pi_right(s_rkp, s_rki); break;
    case PARAM_RIGHT_KI: s_rki = v; control_set_pi_right(s_rkp, s_rki); break;
    case PARAM_RIGHT_KFF: control_set_kff_right(v);                     break;

    case PARAM_LINE_KP: s_line_kp = v; nav_line_set_gains(s_line_kp, s_line_ki, s_line_kd); break;
    case PARAM_LINE_KI: s_line_ki = v; nav_line_set_gains(s_line_kp, s_line_ki, s_line_kd); break;
    case PARAM_LINE_KD: s_line_kd = v; nav_line_set_gains(s_line_kp, s_line_ki, s_line_kd); break;
    case PARAM_LINE_CRUISE_MPS:      nav_line_set_cruise_mps(v);      break;
    case PARAM_QTR_LINE_LOST_THRESH: nav_line_set_lost_threshold(v);  break;
    case PARAM_LINE_T_BLACK:         nav_line_set_t_black_counts(v);  break;

    case PARAM_WEIGHT_CAUTION_KG: safety_set_weight_caution_kg(v);  break;
    case PARAM_WEIGHT_ESTOP_KG:   safety_set_weight_estop_kg(v);    break;
    case PARAM_IMBALANCE_CAUTION: safety_set_imbalance_caution(v);  break;
    case PARAM_IMBALANCE_ESTOP:   safety_set_imbalance_estop(v);    break;

    case PARAM_MAX_LINEAR_ACCEL:  ramp_set_max_accel_lin(v); break;
    case PARAM_MAX_ANGULAR_ACCEL: ramp_set_max_accel_ang(v); break;
    case PARAM_RAMP_SHAPE:        ramp_set_shape((ramp_shape_t)(int)v); break;
    case PARAM_RAMP_JERK_LIN:     ramp_set_max_jerk_lin(v); break;
    case PARAM_RAMP_JERK_ANG:     ramp_set_max_jerk_ang(v); break;
    case PARAM_RAMP_TAU_LIN:      ramp_set_tau_lin(v); break;
    case PARAM_RAMP_TAU_ANG:      ramp_set_tau_ang(v); break;

    case PARAM_LED_MODE:          telemetry_set_led_mode((uint8_t)v); break;
    case PARAM_LED_BASE:
        if (v != 0.0f) s_indicator_cfg |=  (uint8_t)(1u << LED_IND_BASE_BIT);
        else           s_indicator_cfg &= (uint8_t)~(1u << LED_IND_BASE_BIT);
        telemetry_set_indicator_cfg(s_indicator_cfg);
        break;
    case PARAM_LED_INDICATOR_MODE:
        if (v != 0.0f) s_indicator_cfg |=  (uint8_t)(1u << LED_IND_MODE_BIT);
        else           s_indicator_cfg &= (uint8_t)~(1u << LED_IND_MODE_BIT);
        telemetry_set_indicator_cfg(s_indicator_cfg);
        break;

    case PARAM_BATT_3S_CAUTION_MV: safety_set_battery_caution_mv(v); break;
    case PARAM_BATT_3S_ESTOP_MV:   safety_set_battery_estop_mv(v);   break;
    case PARAM_LIDAR_CAUTION_MM:   safety_set_lidar_caution_mm(v);   break;
    case PARAM_LIDAR_CRITICAL_MM:  safety_set_lidar_critical_mm(v);  break;
    case PARAM_LIDAR_ESTOP_MM:     safety_set_lidar_estop_mm(v);     break;

    /* Recognised on the wire but no runtime setter yet — dropped on purpose. */
    case PARAM_MAX_LINEAR_SPEED:
    case PARAM_MAX_ANGULAR_SPEED:
    case PARAM_TELEMETRY_RATE_HZ:
    case PARAM_HEARTBEAT_TIMEOUT_MS:
        return false;

    default:
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_PARAM_ID_UNKNOWN, id);
        return false;
    }
    return true;
}

static uint32_t param_apply_payload(const uint8_t *payload, uint32_t len) {
    uint32_t failed = 0;
    for (uint32_t off = 0; off + 5u <= len; off += 5u) {
        float v;
        memcpy(&v, &payload[off + 1u], sizeof v);
        if (!param_apply_one(payload[off], v)) failed++;
    }
    return failed;
}

/* ===========================================================================
 *  Command dispatch
 * =========================================================================== */

static void handle_cmd(uint8_t seq, const uint8_t *p, uint8_t len) {
    if (len < 1) { send_nack(seq, NACK_BAD_LENGTH); return; }
    uint8_t sub = p[0];

    switch (sub) {
    case CMD_SET_MODE:
        if (len < 2) { send_nack(seq, NACK_BAD_LENGTH); return; }
        if (safety_set_mode((agv_mode_t)p[1])) send_ack(seq, 0);
        else                                   send_nack(seq, NACK_ILLEGAL_TRANSITION);
        return;

    case CMD_SET_FUNCTION:
        if (len < 2) { send_nack(seq, NACK_BAD_LENGTH); return; }
        if (safety_set_function((agv_function_t)p[1])) send_ack(seq, 0);
        else                                           send_nack(seq, NACK_ILLEGAL_TRANSITION);
        return;

    case CMD_VEL_CMD: {
        if (len < 1u + 2u * sizeof(float)) { send_nack(seq, NACK_BAD_LENGTH); return; }
        float v, w;
        memcpy(&v, &p[1], sizeof v);
        memcpy(&w, &p[1 + sizeof(float)], sizeof w);
        nav_remote_set(v, w);
        send_ack(seq, 0);
        return;
    }

    case CMD_VIRTUAL_ESTOP:
        safety_estop_assert(ESTOP_SRC_WORKSTATION);
        send_ack(seq, 0);
        return;

    case CMD_OVERRIDE_ESTOP_SOURCE: {
        if (len < 2) { send_nack(seq, NACK_BAD_LENGTH); return; }
        /* u16 mask, little-endian; the optional high byte reaches battery/LiDAR. */
        uint16_t mask = p[1];
        if (len >= 3) mask |= (uint16_t)p[2] << 8;
        safety_estop_force_clear(mask);
        send_ack(seq, 0);
        return;
    }

    case CMD_OVERRIDE_CAUTION: {
        if (len < 1u + sizeof(float)) { send_nack(seq, NACK_BAD_LENGTH); return; }
        float s;
        memcpy(&s, &p[1], sizeof s);
        safety_caution_set_ws_override(s);
        send_ack(seq, 0);
        return;
    }

    case CMD_START_TARE:
        if (safety_function() != FUNC_STANDBY) { send_nack(seq, NACK_ILLEGAL_TRANSITION); return; }
        loadcells_start_tare();
        send_ack(seq, 0);
        return;

    case CMD_LOG_DUMP_REQUEST:   /* logs already stream continuously */
        send_ack(seq, 0);
        return;

    case CMD_LOG_CLEAR:
        log_clear();
        send_ack(seq, 0);
        return;

    case CMD_LOAD_RAMP_CURVE: {
        if (len < 2) { send_nack(seq, NACK_BAD_LENGTH); return; }
        uint8_t op = p[1];
        if (op == 0) {
            ramp_curve_begin();
            send_ack(seq, 0);
        } else if (op == 1) {
            if (len < 2u + 2u * sizeof(float)) { send_nack(seq, NACK_BAD_LENGTH); return; }
            float s, f;
            memcpy(&s, &p[2], sizeof s);
            memcpy(&f, &p[2 + sizeof(float)], sizeof f);
            ramp_curve_add_point(s, f) ? send_ack(seq, 0) : send_nack(seq, NACK_BUSY);
        } else if (op == 2) {
            ramp_curve_commit() ? send_ack(seq, 0) : send_nack(seq, NACK_BAD_LENGTH);
        } else if (op == 3) {
            ramp_curve_cancel();
            send_ack(seq, 0);
        } else {
            send_nack(seq, NACK_UNKNOWN_SUBTYPE);
        }
        return;
    }

    case CMD_RESET_ODOMETRY:
        if (safety_function() != FUNC_STANDBY) { send_nack(seq, NACK_ILLEGAL_TRANSITION); return; }
        odometry_reset();
        send_ack(seq, 0);
        return;

    case CMD_READ_SENSOR:   /* telemetry covers everything; deferred */
    default:
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UNKNOWN_CMD_SUBTYPE, sub);
        send_nack(seq, NACK_UNKNOWN_SUBTYPE);
        return;
    }
}

static void dispatch(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len) {
    /* Any packet from the workstation is proof of life — except the two pushed
     * autonomously from nearer hops (LIDAR_SEGMENTS by the Jetson, BATTERY by
     * the ESP32): they must not mask a dead workstation (the heartbeat is
     * end-to-end, GUI → firmware). */
    if (type != PKT_LIDAR_SEGMENTS && type != PKT_BATTERY)
        safety_heartbeat_received();

    switch (type) {
    case PKT_HEARTBEAT:
        send_ack(seq, 0);
        return;

    case PKT_CMD:
        handle_cmd(seq, payload, len);
        return;

    case PKT_PARAM_UPDATE:
        if (param_apply_payload(payload, len) == 0) send_ack(seq, 0);
        else                                        send_nack(seq, NACK_UNKNOWN_PARAM);
        return;

    case PKT_LIDAR_SEGMENTS: {
        /* Jetson-segmented LaserScan distances, LE u16 mm each. Fire-and-forget
         * (no ACK), like cmd_vel. Assemble byte-wise: M0 faults on unaligned u16. */
        uint16_t seg[LIDAR_MAX_SEGMENTS];
        uint8_t n = (uint8_t)(len / 2u);
        if (n > LIDAR_MAX_SEGMENTS) n = LIDAR_MAX_SEGMENTS;
        for (uint8_t i = 0; i < n; i++)
            seg[i] = (uint16_t)(payload[2u * i] | ((uint16_t)payload[2u * i + 1u] << 8));
        lidar_set_segments(seg, n);
        return;
    }

    case PKT_BATTERY:
        /* ESP32-pushed INA219 reading; fire-and-forget like LIDAR_SEGMENTS. */
        if (len >= 2u)
            battery_push_mv((uint16_t)(payload[0] | ((uint16_t)payload[1] << 8)));
        return;

    case PKT_RESET: {
        uint8_t mode = (len > 0) ? payload[0] : 0u;
        if (mode == 1u) {
            send_ack(seq, 0);
            log_record(LOG_MOD_SYSTEM, LOG_SEV_WARN, LOG_CODE_SOFT_RESET, 1);
            /* Let the ACK finish on the wire before tearing down peripherals. */
            uint32_t until = mcu_now_ms() + 20u;
            while ((int32_t)(until - mcu_now_ms()) > 0) { }
            mcu_soft_reset();
        } else {
            safety_estop_clear_all();
            send_ack(seq, 0);
        }
        return;
    }

    case PKT_NACK:
        if (len >= 2)
            log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_REMOTE_NACK,
                       ((uint32_t)payload[0] << 8) | payload[1]);
        return;

    case PKT_ACK:
        return;   /* outbound is fire-and-forget; nothing to reconcile */

    default:
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UNKNOWN_PACKET_TYPE, type);
        send_nack(seq, NACK_UNKNOWN_TYPE);
        return;
    }
}

/* ===========================================================================
 *  Inbound frame parser (fed from the UART RX ring)
 * =========================================================================== */

typedef enum {
    P_MAGIC0, P_MAGIC1, P_VER, P_SEQ, P_TYPE, P_LEN, P_PAYLOAD, P_CRC_LO, P_CRC_HI
} pstate_t;

static pstate_t s_state = P_MAGIC0;
static uint8_t  s_ver, s_seq, s_type, s_len;
static uint8_t  s_payload[PROTO_MAX_PAYLOAD];
static uint16_t s_pidx = 0;
static uint16_t s_crc_rx = 0;

static bool     s_seq_seen = false;
static uint8_t  s_last_seq = 0;

static void parser_reset(void) { s_state = P_MAGIC0; s_pidx = 0; }

static void on_frame_complete(void) {
    /* SEQ gap detection (diagnostic). PKT_BATTERY is exempt: the ESP32 injects
     * it with its own SEQ counter, so it would alias as a gap against the host
     * stream (and the host's next frame as another) on every 2 Hz push. */
    if (s_type != PKT_BATTERY) {
        if (s_seq_seen) {
            uint8_t expected = (uint8_t)(s_last_seq + 1u);
            if (s_seq != expected)
                log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_SEQ_GAP,
                           (uint32_t)(uint8_t)(s_seq - expected));
        }
        s_last_seq = s_seq;
        s_seq_seen = true;
    }

    dispatch(s_type, s_seq, s_payload, s_len);
}

static void parser_feed(uint8_t b) {
    switch (s_state) {
    case P_MAGIC0:
        if (b == PROTO_MAGIC0) s_state = P_MAGIC1;
        break;
    case P_MAGIC1:
        if (b == PROTO_MAGIC1) s_state = P_VER;
        else { s_state = (b == PROTO_MAGIC0) ? P_MAGIC1 : P_MAGIC0; }
        break;
    case P_VER:
        s_ver = b;
        if (b != PROTO_VERSION) {
            log_record(LOG_MOD_COMMS, LOG_SEV_ERROR, LOG_CODE_BAD_VERSION, b);
            parser_reset();
        } else {
            s_state = P_SEQ;
        }
        break;
    case P_SEQ:  s_seq  = b; s_state = P_TYPE; break;
    case P_TYPE: s_type = b; s_state = P_LEN;  break;
    case P_LEN:
        s_len = b;
        if (s_len == 0) { s_state = P_CRC_LO; }
        else            { s_pidx = 0; s_state = P_PAYLOAD; }
        break;
    case P_PAYLOAD:
        s_payload[s_pidx++] = b;
        if (s_pidx >= s_len) s_state = P_CRC_LO;
        break;
    case P_CRC_LO:
        s_crc_rx = b;
        s_state = P_CRC_HI;
        break;
    case P_CRC_HI: {
        s_crc_rx |= ((uint16_t)b << 8);
        uint8_t tmp[4 + PROTO_MAX_PAYLOAD];
        tmp[0] = s_ver; tmp[1] = s_seq; tmp[2] = s_type; tmp[3] = s_len;
        if (s_len) memcpy(&tmp[4], s_payload, s_len);
        uint16_t crc = crc16_ccitt(tmp, 4u + s_len);
        if (crc == s_crc_rx) {
            on_frame_complete();
        } else {
            log_record(LOG_MOD_COMMS, LOG_SEV_ERROR, LOG_CODE_BAD_CRC,
                       ((uint32_t)s_crc_rx << 16) | crc);
            send_nack(s_seq, NACK_BAD_CRC);
        }
        parser_reset();
        break;
    }
    }
}

/* ===========================================================================
 *  UART line-error surfacing
 * =========================================================================== */

static uint16_t s_seen_overrun = 0, s_seen_framing = 0, s_seen_noise = 0;

static void surface_uart_errors(void) {
    uint16_t o = uart_err_overrun(), f = uart_err_framing(), n = uart_err_noise();
    if (o != s_seen_overrun) { log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UART_OVERRUN, o);     s_seen_overrun = o; }
    if (f != s_seen_framing) { log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UART_FRAMING_ERR, f); s_seen_framing = f; }
    if (n != s_seen_noise)   { log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UART_NOISE, n);       s_seen_noise = n; }
}

/* ===========================================================================
 *  Public
 * =========================================================================== */

void proto_init(void) {
    uart_init();
    parser_reset();
    s_seq_seen = false;
}

void proto_poll(void) {
    uint8_t b;
    while (uart_rx_pop(&b)) parser_feed(b);
    surface_uart_errors();
}
