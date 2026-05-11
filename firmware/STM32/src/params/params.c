#include "params.h"
#include "cargo_monitor.h"
#include "comms.h"
#include "control.h"
#include "hx711.h"
#include "log.h"
#include "nav_line.h"
#include "nav_traj.h"
#include "types.h"
#include <string.h>

/* Live-mutable mirror state for params that aren't directly owned by another
 * module's state. (PID gains live in control.c; HX711 cal lives in hx711.c;
 * weight thresholds aren't currently runtime-checked elsewhere — these are
 * just hooks, the cargo monitor still reads from config.h. If you need them
 * truly mutable, mirror them here and have cargo_monitor consult.)
 *
 * For now we wire what's actually consumed at runtime; the rest log + skip.
 */

/* Per-axis PID gain accumulators — we receive Kp/Ki/Kd as separate PARAM_UPDATE
 * entries, so we keep current values and apply the triple together when any
 * axis-component arrives. Initialised from config.h defaults. */
#include "config.h"

typedef struct { float kp, ki, kd; } gains_t;

static gains_t s_inner_left  = { PID_INNER_KP_LEFT,  PID_INNER_KI_LEFT,  PID_INNER_KD_LEFT  };
static gains_t s_inner_right = { PID_INNER_KP_RIGHT, PID_INNER_KI_RIGHT, PID_INNER_KD_RIGHT };
static gains_t s_outer_lin   = { PID_OUTER_LIN_KP,   PID_OUTER_LIN_KI,   PID_OUTER_LIN_KD   };
static gains_t s_outer_ang   = { PID_OUTER_ANG_KP,   PID_OUTER_ANG_KI,   PID_OUTER_ANG_KD   };

bool params_apply_one(uint8_t id, float v) {
    /* HX711 corner-indexed params decode the corner from the low nibble. */
    if (id >= PARAM_HX711_OFFSET_BASE && id < PARAM_HX711_OFFSET_BASE + 4u) {
        uint32_t corner = id - PARAM_HX711_OFFSET_BASE;
        hx711_set_offset(corner, (int32_t)v);
        log_record(LOG_MOD_HX711, LOG_SEV_INFO, LOG_CODE_PARAM_APPLIED, id);
        return true;
    }
    if (id >= PARAM_HX711_SCALE_BASE && id < PARAM_HX711_SCALE_BASE + 4u) {
        uint32_t corner = id - PARAM_HX711_SCALE_BASE;
        hx711_set_scale(corner, v);
        log_record(LOG_MOD_HX711, LOG_SEV_INFO, LOG_CODE_PARAM_APPLIED, id);
        return true;
    }

    switch (id) {
    /* Inner-loop gains — buffer locally, push the full triple after each
     * component arrives so partial updates don't leave the PID in a weird
     * state. (If only Kp arrives, Ki/Kd retain their previous values.) */
    case PARAM_INNER_KP_LEFT:  s_inner_left.kp  = v;
        control_set_inner_gains_left(s_inner_left.kp, s_inner_left.ki, s_inner_left.kd); break;
    case PARAM_INNER_KI_LEFT:  s_inner_left.ki  = v;
        control_set_inner_gains_left(s_inner_left.kp, s_inner_left.ki, s_inner_left.kd); break;
    case PARAM_INNER_KD_LEFT:  s_inner_left.kd  = v;
        control_set_inner_gains_left(s_inner_left.kp, s_inner_left.ki, s_inner_left.kd); break;
    case PARAM_INNER_KP_RIGHT: s_inner_right.kp = v;
        control_set_inner_gains_right(s_inner_right.kp, s_inner_right.ki, s_inner_right.kd); break;
    case PARAM_INNER_KI_RIGHT: s_inner_right.ki = v;
        control_set_inner_gains_right(s_inner_right.kp, s_inner_right.ki, s_inner_right.kd); break;
    case PARAM_INNER_KD_RIGHT: s_inner_right.kd = v;
        control_set_inner_gains_right(s_inner_right.kp, s_inner_right.ki, s_inner_right.kd); break;
    case PARAM_OUTER_LIN_KP:   s_outer_lin.kp   = v;
        control_set_outer_lin_gains(s_outer_lin.kp, s_outer_lin.ki, s_outer_lin.kd); break;
    case PARAM_OUTER_LIN_KI:   s_outer_lin.ki   = v;
        control_set_outer_lin_gains(s_outer_lin.kp, s_outer_lin.ki, s_outer_lin.kd); break;
    case PARAM_OUTER_LIN_KD:   s_outer_lin.kd   = v;
        control_set_outer_lin_gains(s_outer_lin.kp, s_outer_lin.ki, s_outer_lin.kd); break;
    case PARAM_OUTER_ANG_KP:   s_outer_ang.kp   = v;
        control_set_outer_ang_gains(s_outer_ang.kp, s_outer_ang.ki, s_outer_ang.kd); break;
    case PARAM_OUTER_ANG_KI:   s_outer_ang.ki   = v;
        control_set_outer_ang_gains(s_outer_ang.kp, s_outer_ang.ki, s_outer_ang.kd); break;
    case PARAM_OUTER_ANG_KD:   s_outer_ang.kd   = v;
        control_set_outer_ang_gains(s_outer_ang.kp, s_outer_ang.ki, s_outer_ang.kd); break;

    /* Navigator tunables — exposed in the dashboard function panels. */
    case PARAM_LINE_CRUISE_MPS:        nav_line_set_cruise_mps(v);          break;
    case PARAM_QTR_LINE_LOST_THRESH:   nav_line_set_lost_threshold(v);      break;
    case PARAM_TRAJ_CRUISE_MPS:        nav_traj_set_cruise_mps(v);          break;
    case PARAM_TRAJ_LOOKAHEAD_M:       nav_traj_set_lookahead_m(v);         break;

    /* Cargo thresholds. */
    case PARAM_WEIGHT_CAUTION_KG:      cargo_monitor_set_caution_kg(v);     break;
    case PARAM_WEIGHT_ESTOP_KG:        cargo_monitor_set_estop_kg(v);       break;
    case PARAM_IMBALANCE_CAUTION:      cargo_monitor_set_imbalance_caution(v); break;
    case PARAM_IMBALANCE_ESTOP:        cargo_monitor_set_imbalance_estop(v);   break;

    /* Recognised but not yet plumbed — modules still read these from config.h.
     * Wire them through the relevant module if/when you need live tuning. */
    case PARAM_MAX_LINEAR_SPEED:
    case PARAM_MAX_ANGULAR_SPEED:
    case PARAM_MAX_LINEAR_ACCEL:
    case PARAM_MAX_ANGULAR_ACCEL:
    case PARAM_TELEMETRY_RATE_HZ:
    case PARAM_HEARTBEAT_TIMEOUT_MS:
    case PARAM_LINE_KP:
    case PARAM_LINE_KI:
    case PARAM_LINE_KD:
        log_record(LOG_MOD_COMMS, LOG_SEV_INFO, LOG_CODE_PARAM_APPLIED, id);
        return false;   /* recognised but not yet plumbed through */

    default:
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_PARAM_ID_UNKNOWN, id);
        return false;
    }

    log_record(LOG_MOD_COMMS, LOG_SEV_INFO, LOG_CODE_PARAM_APPLIED, id);
    return true;
}

uint32_t params_apply_payload(const uint8_t *payload, uint32_t len) {
    uint32_t failed = 0;
    /* Each tuple = 1 byte id + 4 bytes f32. */
    for (uint32_t off = 0; off + 5u <= len; off += 5u) {
        uint8_t id = payload[off];
        float v;
        memcpy(&v, &payload[off + 1u], sizeof v);
        if (!params_apply_one(id, v)) failed++;
    }
    return failed;
}
