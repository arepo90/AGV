#include "params.h"
#include "cargo_monitor.h"
#include "comms.h"
#include "control.h"
#include "hx711.h"
#include "log.h"
#include "nav_line.h"
#include "nav_traj.h"
#include "ramp.h"
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

/* Per-axis gain accumulators. STA loops (cascade) take (k1, k2); the line
 * navigator still runs a classical PID and takes (kp, ki, kd). We mirror the
 * current values so partial PARAM_UPDATE batches (just k1, just k2) still
 * push a consistent pair to the controller. Initialised from config.h. */
#include "config.h"

typedef struct { float k1, k2; }      sta_gains_t;
typedef struct { float kp, ki, kd; }  pid_gains_t;

static sta_gains_t s_inner_left  = { STA_INNER_K1_LEFT,  STA_INNER_K2_LEFT  };
static sta_gains_t s_inner_right = { STA_INNER_K1_RIGHT, STA_INNER_K2_RIGHT };
static sta_gains_t s_outer_lin   = { STA_OUTER_LIN_K1,   STA_OUTER_LIN_K2   };
static sta_gains_t s_outer_ang   = { STA_OUTER_ANG_K1,   STA_OUTER_ANG_K2   };
static pid_gains_t s_line        = { LINE_FOLLOW_KP,     LINE_FOLLOW_KI,     LINE_FOLLOW_KD };

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
    /* STA cascade gains — buffer locally, push the (k1, k2) pair after each
     * component arrives so partial updates don't leave the controller with a
     * stale k2 when only k1 is sent. */
    case PARAM_INNER_K1_LEFT:  s_inner_left.k1  = v;
        control_set_inner_gains_left(s_inner_left.k1, s_inner_left.k2); break;
    case PARAM_INNER_K2_LEFT:  s_inner_left.k2  = v;
        control_set_inner_gains_left(s_inner_left.k1, s_inner_left.k2); break;
    case PARAM_INNER_K1_RIGHT: s_inner_right.k1 = v;
        control_set_inner_gains_right(s_inner_right.k1, s_inner_right.k2); break;
    case PARAM_INNER_K2_RIGHT: s_inner_right.k2 = v;
        control_set_inner_gains_right(s_inner_right.k1, s_inner_right.k2); break;
    case PARAM_OUTER_LIN_K1:   s_outer_lin.k1   = v;
        control_set_outer_lin_gains(s_outer_lin.k1, s_outer_lin.k2); break;
    case PARAM_OUTER_LIN_K2:   s_outer_lin.k2   = v;
        control_set_outer_lin_gains(s_outer_lin.k1, s_outer_lin.k2); break;
    case PARAM_OUTER_ANG_K1:   s_outer_ang.k1   = v;
        control_set_outer_ang_gains(s_outer_ang.k1, s_outer_ang.k2); break;
    case PARAM_OUTER_ANG_K2:   s_outer_ang.k2   = v;
        control_set_outer_ang_gains(s_outer_ang.k1, s_outer_ang.k2); break;

    /* Navigator tunables — exposed in the dashboard function panels. */
    case PARAM_LINE_KP:                s_line.kp = v;
        nav_line_set_gains(s_line.kp, s_line.ki, s_line.kd); break;
    case PARAM_LINE_KI:                s_line.ki = v;
        nav_line_set_gains(s_line.kp, s_line.ki, s_line.kd); break;
    case PARAM_LINE_KD:                s_line.kd = v;
        nav_line_set_gains(s_line.kp, s_line.ki, s_line.kd); break;
    case PARAM_LINE_CRUISE_MPS:        nav_line_set_cruise_mps(v);          break;
    case PARAM_QTR_LINE_LOST_THRESH:   nav_line_set_lost_threshold(v);      break;
    case PARAM_TRAJ_CRUISE_MPS:        nav_traj_set_cruise_mps(v);          break;
    case PARAM_TRAJ_LOOKAHEAD_M:       nav_traj_set_lookahead_m(v);         break;
    case PARAM_TRAJ_CURV_SLOWDOWN:     nav_traj_set_curv_slowdown(v);       break;

    /* Cargo thresholds. */
    case PARAM_WEIGHT_CAUTION_KG:      cargo_monitor_set_caution_kg(v);     break;
    case PARAM_WEIGHT_ESTOP_KG:        cargo_monitor_set_estop_kg(v);       break;
    case PARAM_IMBALANCE_CAUTION:      cargo_monitor_set_imbalance_caution(v); break;
    case PARAM_IMBALANCE_ESTOP:        cargo_monitor_set_imbalance_estop(v);   break;

    /* Ramp / motion profile. */
    case PARAM_MAX_LINEAR_ACCEL:       ramp_set_max_accel_lin(v); break;
    case PARAM_MAX_ANGULAR_ACCEL:      ramp_set_max_accel_ang(v); break;
    case PARAM_RAMP_SHAPE:             ramp_set_shape((ramp_shape_t)(int)v); break;
    case PARAM_RAMP_JERK_LIN:          ramp_set_max_jerk_lin(v); break;
    case PARAM_RAMP_JERK_ANG:          ramp_set_max_jerk_ang(v); break;
    case PARAM_RAMP_TAU_LIN:           ramp_set_tau_lin(v); break;
    case PARAM_RAMP_TAU_ANG:           ramp_set_tau_ang(v); break;

    /* Recognised by the protocol but the owning module has no runtime setter
     * yet, so the value is dropped on the floor:
     *   MAX_LINEAR/ANGULAR_SPEED  control.c hard-reads MAX_*_MPS macros
     *   TELEMETRY_RATE_HZ         main.c uses two separate moving/standby rates
     *   HEARTBEAT_TIMEOUT_MS      heartbeat.c hard-reads HEARTBEAT_TIMEOUT_MS
     * To make any of these tunable, add a setter on the owning module and a
     * case above that calls it. */
    case PARAM_MAX_LINEAR_SPEED:
    case PARAM_MAX_ANGULAR_SPEED:
    case PARAM_TELEMETRY_RATE_HZ:
    case PARAM_HEARTBEAT_TIMEOUT_MS:
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
