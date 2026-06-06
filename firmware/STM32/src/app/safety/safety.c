#include "safety.h"
#include "analog.h"
#include "battery.h"
#include "config.h"
#include "lidar.h"
#include "loadcells.h"
#include "log.h"
#include "mcu.h"
#include "stm32f0xx.h"
#include "tof.h"
#include <math.h>

/* ===========================================================================
 *  Mode / function state machine
 * =========================================================================== */

static agv_mode_t     s_mode = MODE_SUPERVISED;
static agv_function_t s_func = FUNC_STANDBY;

bool safety_function_is_navigating(agv_function_t f) { return f != FUNC_STANDBY; }
static bool function_is_supervised_only(agv_function_t f) { return f == FUNC_REMOTE_CONTROL; }

agv_mode_t     safety_mode(void)     { return s_mode; }
agv_function_t safety_function(void) { return s_func; }

/* UNSUPERVISED + navigating → CAUTION baseline (architecture §Caution). */
static void update_unsupervised_baseline(void) {
    if (s_mode == MODE_UNSUPERVISED && safety_function_is_navigating(s_func))
        safety_caution_set(CAUTION_SRC_UNSUPERVISED_NAV, CAUTION_LEVEL_CAUTION);
    else
        safety_caution_clear(CAUTION_SRC_UNSUPERVISED_NAV);
}

bool safety_set_mode(agv_mode_t m) {
    if (m == s_mode) return true;
    if (m != MODE_SUPERVISED && m != MODE_UNSUPERVISED) {
        log_record(LOG_MOD_STATE, LOG_SEV_ERROR, LOG_CODE_ILLEGAL_TRANSITION,
                   ((uint32_t)s_mode << 8) | (uint32_t)m);
        return false;
    }
    agv_mode_t old = s_mode;
    s_mode = m;
    log_record(LOG_MOD_STATE, LOG_SEV_INFO, LOG_CODE_MODE_TRANSITION,
               ((uint32_t)old << 8) | (uint32_t)m);

    /* Downgrade a SUPERVISED-only function when leaving SUPERVISED. */
    if (s_mode == MODE_UNSUPERVISED && function_is_supervised_only(s_func)) {
        agv_function_t of = s_func;
        s_func = FUNC_STANDBY;
        log_record(LOG_MOD_STATE, LOG_SEV_WARN, LOG_CODE_FUNCTION_TRANSITION,
                   ((uint32_t)of << 8) | (uint32_t)s_func);
    }
    update_unsupervised_baseline();
    return true;
}

bool safety_set_function(agv_function_t f) {
    if (f == s_func) return true;
    if (f != FUNC_STANDBY && f != FUNC_REMOTE_CONTROL &&
        f != FUNC_LINE_FOLLOW && f != FUNC_TRAJECTORY_FOLLOW) {
        log_record(LOG_MOD_STATE, LOG_SEV_ERROR, LOG_CODE_ILLEGAL_TRANSITION,
                   ((uint32_t)s_func << 8) | (uint32_t)f);
        return false;
    }
    if (function_is_supervised_only(f) && s_mode != MODE_SUPERVISED) {
        log_record(LOG_MOD_STATE, LOG_SEV_ERROR, LOG_CODE_ILLEGAL_TRANSITION,
                   ((uint32_t)s_func << 8) | (uint32_t)f);
        return false;
    }
    agv_function_t of = s_func;
    s_func = f;
    log_record(LOG_MOD_STATE, LOG_SEV_INFO, LOG_CODE_FUNCTION_TRANSITION,
               ((uint32_t)of << 8) | (uint32_t)f);
    update_unsupervised_baseline();
    return true;
}

/* ===========================================================================
 *  Virtual E-STOP
 * =========================================================================== */

static volatile uint16_t s_estop = 0;   /* 16-bit: room past 8 sources (see types.h) */

static void estop_log(estop_source_t src, bool set) {
    log_record(LOG_MOD_ESTOP,
               set ? LOG_SEV_CRITICAL : LOG_SEV_INFO,
               set ? LOG_CODE_ESTOP_ASSERTED : LOG_CODE_ESTOP_CLEARED,
               (uint32_t)src);
}

void safety_estop_assert(estop_source_t src) {
    if (src == 0) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool was = (s_estop & (uint16_t)src) != 0;
    s_estop |= (uint16_t)src;
    if (!primask) __enable_irq();
    if (!was) estop_log(src, true);
}

void safety_estop_clear_autoclearing(estop_source_t src) {
    uint16_t allowed = (uint16_t)src & (uint16_t)ESTOP_AUTOCLEAR_MASK;
    if (!allowed) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool was = (s_estop & allowed) != 0;
    s_estop &= (uint16_t)~allowed;
    if (!primask) __enable_irq();
    if (was) estop_log(src, false);
}

void safety_estop_force_clear(uint16_t mask) {
    if (!mask) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t cleared = s_estop & mask;
    s_estop &= (uint16_t)~mask;
    if (!primask) __enable_irq();
    if (cleared)
        log_record(LOG_MOD_ESTOP, LOG_SEV_WARN, LOG_CODE_ESTOP_OVERRIDE, cleared);
}

void     safety_estop_clear_all(void) { safety_estop_force_clear(0xFFFFu); }
bool     safety_estop_active(void)    { return s_estop != 0; }
uint16_t safety_estop_sources(void)   { return s_estop; }

/* ===========================================================================
 *  Caution modifier — per-source minimum, with workstation override
 * =========================================================================== */

#define CAUTION_MAX_SRC 8u

static float s_level[CAUTION_MAX_SRC];
static float s_ws_override = 1.0f;
static bool  s_ws_override_active = false;

static int caution_index(caution_source_t src) {
    if (src == 0) return -1;
    int i = 0;
    while (((uint32_t)src & 1u) == 0u) { src = (caution_source_t)((uint32_t)src >> 1); i++; }
    return (i < (int)CAUTION_MAX_SRC) ? i : -1;
}

void safety_caution_set(caution_source_t src, float level) {
    if (src == CAUTION_SRC_WORKSTATION_FORCED) return;   /* owned by the override path */
    int i = caution_index(src);
    if (i < 0) return;
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    s_level[i] = level;   /* aligned float store is atomic on M0 */
}

void safety_caution_clear(caution_source_t src) { safety_caution_set(src, CAUTION_NORMAL); }

void safety_caution_set_ws_override(float level) {
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    s_ws_override = level;
    s_ws_override_active = true;
}

float safety_caution_modifier(void) {
    if (s_ws_override_active) return s_ws_override;   /* full authority */
    float m = CAUTION_NORMAL;
    for (uint32_t i = 0; i < CAUTION_MAX_SRC; i++)
        if (s_level[i] < m) m = s_level[i];
    return m;
}

uint16_t safety_caution_sources(void) {
    uint16_t bits = 0;
    for (uint32_t i = 0; i < CAUTION_MAX_SRC; i++)
        if (s_level[i] < CAUTION_NORMAL) bits |= (uint16_t)(1u << i);
    if (s_ws_override_active && s_ws_override < CAUTION_NORMAL)
        bits |= (uint16_t)CAUTION_SRC_WORKSTATION_FORCED;
    return bits;
}

/* ===========================================================================
 *  Heartbeat watchdog — two-stage degradation
 * =========================================================================== */

static uint32_t s_hb_last_ms = 0;
static uint8_t  s_hb_stage = 0;
static bool     s_hb_unsupervised_due_to_timeout = false;

uint8_t safety_heartbeat_stage(void) { return s_hb_stage; }

void safety_heartbeat_received(void) {
    s_hb_last_ms = mcu_now_ms();
    if (s_hb_stage > 0) {
        log_record(LOG_MOD_HEARTBEAT, LOG_SEV_INFO, LOG_CODE_HEARTBEAT_RESTORED, s_hb_stage);
        s_hb_stage = 0;
        /* Restore SUPERVISED if we degraded ourselves. The grace E-STOP is NOT
         * auto-cleared — the workstation must clear it explicitly. */
        if (s_hb_unsupervised_due_to_timeout && s_mode == MODE_UNSUPERVISED)
            safety_set_mode(MODE_SUPERVISED);
        s_hb_unsupervised_due_to_timeout = false;
    }
}

void safety_heartbeat_tick(void) {
#if DISABLE_HEARTBEAT_WATCH
    return;
#else
    /* Dormant in UNSUPERVISED unless we are the ones who degraded the mode. */
    if (s_mode == MODE_UNSUPERVISED && !s_hb_unsupervised_due_to_timeout) {
        s_hb_last_ms = mcu_now_ms();
        return;
    }
    uint32_t elapsed = mcu_now_ms() - s_hb_last_ms;

    if (s_hb_stage == 0 && elapsed > HEARTBEAT_TIMEOUT_MS) {
        s_hb_stage = 1;
        log_record(LOG_MOD_HEARTBEAT, LOG_SEV_WARN, LOG_CODE_HEARTBEAT_LOST, elapsed);
        if (s_mode == MODE_SUPERVISED) {
            s_hb_unsupervised_due_to_timeout = true;
            safety_set_mode(MODE_UNSUPERVISED);
        }
    }
    if (s_hb_stage == 1 && elapsed > (HEARTBEAT_TIMEOUT_MS + HEARTBEAT_GRACE_MS)) {
        s_hb_stage = 2;
        log_record(LOG_MOD_HEARTBEAT, LOG_SEV_CRITICAL, LOG_CODE_HEARTBEAT_GRACE_EXPIRED, elapsed);
        safety_estop_assert(ESTOP_SRC_HEARTBEAT_TIMEOUT);
    }
#endif
}

/* ===========================================================================
 *  Sensor-derived monitors
 * =========================================================================== */

#define IMBALANCE_FLOOR_KG  2.0f      /* below this, imbalance is just noise */
#define OVERCURRENT_TICKS   10u       /* consecutive control ticks → trip (~100 ms) */

static float   s_w_caution_kg     = WEIGHT_TOTAL_CAUTION_KG;
static float   s_w_estop_kg       = WEIGHT_TOTAL_ESTOP_KG;
static float   s_imbalance_caution = WEIGHT_IMBALANCE_CAUTION;
static float   s_imbalance_estop   = WEIGHT_IMBALANCE_ESTOP;
static uint8_t s_oc_streak[2];

static uint16_t s_tof_caution_mm  = TOF_CAUTION_MM;
static uint16_t s_tof_critical_mm = TOF_CRITICAL_MM;
static uint16_t s_tof_estop_mm    = TOF_ESTOP_MM;
static uint8_t  s_tof_band        = 0;     /* 0 clear,1 caution,2 critical,3 estop */
static uint16_t s_batt_caution_mv = BATTERY_3S_CAUTION_MV;
static uint16_t s_batt_estop_mv   = BATTERY_3S_ESTOP_MV;
static uint8_t  s_batt_state      = 0;     /* 0 normal,1 caution,2 estop (3S, hysteretic) */
static bool     s_batt6_low       = false;
static uint16_t s_lidar_caution_mm  = LIDAR_CAUTION_MM;
static uint16_t s_lidar_critical_mm = LIDAR_CRITICAL_MM;
static uint16_t s_lidar_estop_mm    = LIDAR_ESTOP_MM;
static uint8_t  s_lidar_band        = 0;   /* 0 clear,1 caution,2 critical,3 estop */

void safety_set_weight_caution_kg(float kg)   { if (kg > 0.0f && kg < 1000.0f) s_w_caution_kg = kg; }
void safety_set_weight_estop_kg(float kg)     { if (kg > 0.0f && kg < 1000.0f) s_w_estop_kg = kg; }
void safety_set_imbalance_caution(float f)    { if (f > 0.0f && f < 1.0f) s_imbalance_caution = f; }
void safety_set_imbalance_estop(float f)      { if (f > 0.0f && f < 1.0f) s_imbalance_estop = f; }

void safety_set_tof_caution_mm(float mm)      { if (mm > 0.0f && mm < 4000.0f) s_tof_caution_mm  = (uint16_t)mm; }
void safety_set_tof_critical_mm(float mm)     { if (mm > 0.0f && mm < 4000.0f) s_tof_critical_mm = (uint16_t)mm; }
void safety_set_tof_estop_mm(float mm)        { if (mm > 0.0f && mm < 4000.0f) s_tof_estop_mm    = (uint16_t)mm; }
void safety_set_battery_caution_mv(float mv)  { if (mv > 5000.0f && mv < 30000.0f) s_batt_caution_mv = (uint16_t)mv; }
void safety_set_battery_estop_mv(float mv)    { if (mv > 5000.0f && mv < 30000.0f) s_batt_estop_mv   = (uint16_t)mv; }

void safety_set_lidar_caution_mm(float mm)    { if (mm > 0.0f && mm < 12000.0f) s_lidar_caution_mm  = (uint16_t)mm; }
void safety_set_lidar_critical_mm(float mm)   { if (mm > 0.0f && mm < 12000.0f) s_lidar_critical_mm = (uint16_t)mm; }
void safety_set_lidar_estop_mm(float mm)      { if (mm > 0.0f && mm < 12000.0f) s_lidar_estop_mm    = (uint16_t)mm; }

static void cargo_tick(void) {
#if !DISABLE_LOAD_CELLS
    if (!loadcells_has_data()) return;

    float corner[4], total = 0.0f;
    for (uint32_t c = 0; c < 4; c++) { corner[c] = loadcells_kg(c); total += corner[c]; }

    /* Total weight. */
    if (total >= s_w_estop_kg) {
        safety_estop_assert(ESTOP_SRC_CARGO_OVERLOAD);
        safety_caution_set(CAUTION_SRC_LOAD_OVERWEIGHT, CAUTION_LEVEL_CRITICAL);
    } else if (total >= s_w_caution_kg) {
        safety_estop_clear_autoclearing(ESTOP_SRC_CARGO_OVERLOAD);
        safety_caution_set(CAUTION_SRC_LOAD_OVERWEIGHT, CAUTION_LEVEL_CAUTION);
    } else {
        safety_estop_clear_autoclearing(ESTOP_SRC_CARGO_OVERLOAD);
        safety_caution_clear(CAUTION_SRC_LOAD_OVERWEIGHT);
    }

    /* Imbalance (ignored under the floor). */
    if (total < IMBALANCE_FLOOR_KG) {
        safety_estop_clear_autoclearing(ESTOP_SRC_CARGO_IMBALANCE);
        safety_caution_clear(CAUTION_SRC_LOAD_IMBALANCE);
        return;
    }
    float mean = total * 0.25f;
    float max_dev = 0.0f;
    for (uint32_t c = 0; c < 4; c++) {
        float d = fabsf(corner[c] - mean);
        if (d > max_dev) max_dev = d;
    }
    float frac = max_dev / mean;
    if (frac >= s_imbalance_estop) {
        safety_estop_assert(ESTOP_SRC_CARGO_IMBALANCE);
        safety_caution_set(CAUTION_SRC_LOAD_IMBALANCE, CAUTION_LEVEL_CRITICAL);
    } else if (frac >= s_imbalance_caution) {
        safety_estop_clear_autoclearing(ESTOP_SRC_CARGO_IMBALANCE);
        safety_caution_set(CAUTION_SRC_LOAD_IMBALANCE, CAUTION_LEVEL_CAUTION);
    } else {
        safety_estop_clear_autoclearing(ESTOP_SRC_CARGO_IMBALANCE);
        safety_caution_clear(CAUTION_SRC_LOAD_IMBALANCE);
    }
#endif
}

static void current_tick(void) {
#if !DISABLE_CURRENT_SENSE
    if (!analog_has_data()) return;
    for (uint32_t s = 0; s < 2; s++) {
        if (analog_current_ma(s) > MOTOR_OVERCURRENT_MA) {
            if (++s_oc_streak[s] == OVERCURRENT_TICKS) {
                log_record(LOG_MOD_MOTORS, LOG_SEV_CRITICAL,
                           s ? LOG_CODE_OVERCURRENT_M2 : LOG_CODE_OVERCURRENT_M1,
                           analog_current_ma(s));
                safety_estop_assert(ESTOP_SRC_OVERCURRENT);
            }
        } else {
            s_oc_streak[s] = 0;
        }
    }
#endif
}

/* TOF distance bands → caution level + auto-clearing E-STOP. The minimum range
 * over all present sensors selects the band, mirroring the IR "any-sensor" rule
 * but graduated. Both caution and E-STOP auto-clear as the obstacle recedes. */
static void tof_tick_monitor(void) {
#if !DISABLE_TOF
    if (!tof_any_present()) return;
    uint16_t d = tof_min_distance_mm();

    uint8_t band = (d < s_tof_estop_mm)    ? 3u
                 : (d < s_tof_critical_mm) ? 2u
                 : (d < s_tof_caution_mm)  ? 1u : 0u;

    switch (band) {
    case 3:
        safety_estop_assert(ESTOP_SRC_TOF);
        safety_caution_set(CAUTION_SRC_TOF_NEAR, CAUTION_LEVEL_CRITICAL);
        break;
    case 2:
        safety_estop_clear_autoclearing(ESTOP_SRC_TOF);
        safety_caution_set(CAUTION_SRC_TOF_NEAR, CAUTION_LEVEL_CRITICAL);
        break;
    case 1:
        safety_estop_clear_autoclearing(ESTOP_SRC_TOF);
        safety_caution_set(CAUTION_SRC_TOF_NEAR, CAUTION_LEVEL_CAUTION);
        break;
    default:
        safety_estop_clear_autoclearing(ESTOP_SRC_TOF);
        safety_caution_clear(CAUTION_SRC_TOF_NEAR);
        break;
    }

    if (band != s_tof_band) {
        if (band == 0)
            log_record(LOG_MOD_TOF, LOG_SEV_INFO, LOG_CODE_TOF_CLEARED, d);
        else if (band > s_tof_band)
            log_record(LOG_MOD_TOF, LOG_SEV_WARN, LOG_CODE_TOF_TRIGGERED,
                       ((uint32_t)band << 16) | d);
        s_tof_band = band;
    }
#endif
}

/* LiDAR distance bands → caution + auto-clearing E-STOP, identical policy to the
 * TOF monitor but fed by Jetson-pushed segments (min over the fresh set). A dead
 * Jetson link reports "clear" (fail-safe), so it can never latch the E-STOP. */
static void lidar_tick_monitor(void) {
#if !DISABLE_LIDAR
    bool fresh = lidar_is_fresh();
    uint16_t d = lidar_min_distance_mm();    /* clear sentinel when stale */

    uint8_t band = (d < s_lidar_estop_mm)    ? 3u
                 : (d < s_lidar_critical_mm) ? 2u
                 : (d < s_lidar_caution_mm)  ? 1u : 0u;

    switch (band) {
    case 3:
        safety_estop_assert(ESTOP_SRC_LIDAR);
        safety_caution_set(CAUTION_SRC_LIDAR_NEAR, CAUTION_LEVEL_CRITICAL);
        break;
    case 2:
        safety_estop_clear_autoclearing(ESTOP_SRC_LIDAR);
        safety_caution_set(CAUTION_SRC_LIDAR_NEAR, CAUTION_LEVEL_CRITICAL);
        break;
    case 1:
        safety_estop_clear_autoclearing(ESTOP_SRC_LIDAR);
        safety_caution_set(CAUTION_SRC_LIDAR_NEAR, CAUTION_LEVEL_CAUTION);
        break;
    default:
        safety_estop_clear_autoclearing(ESTOP_SRC_LIDAR);
        safety_caution_clear(CAUTION_SRC_LIDAR_NEAR);
        break;
    }

    if (band != s_lidar_band) {
        if (band == 0)
            log_record(LOG_MOD_LIDAR, LOG_SEV_INFO,
                       fresh ? LOG_CODE_LIDAR_CLEARED : LOG_CODE_LIDAR_STALE, d);
        else if (band > s_lidar_band)
            log_record(LOG_MOD_LIDAR, LOG_SEV_WARN, LOG_CODE_LIDAR_TRIGGERED,
                       ((uint32_t)band << 16) | d);
        s_lidar_band = band;
    }
#endif
}

/* Battery: 3S rail (motors/logic) gets a hysteretic caution→E-STOP state machine
 * so sag-under-load can't chatter; E-STOP auto-clears once recovered. The 6S rail
 * powers only the Jetson, so it is log-warning only. */
static void battery_tick_monitor(void) {
#if !DISABLE_BATTERY
    if (battery_present(BATTERY_3S)) {
        uint16_t mv = battery_mv(BATTERY_3S);
        uint8_t st = s_batt_state;
        if (st == 0) {
            if (mv < s_batt_estop_mv)        st = 2;
            else if (mv < s_batt_caution_mv) st = 1;
        } else if (st == 1) {
            if (mv < s_batt_estop_mv)                          st = 2;
            else if (mv >= s_batt_caution_mv + BATTERY_RECOVER_MV) st = 0;
        } else { /* st == 2 */
            if (mv >= s_batt_caution_mv + BATTERY_RECOVER_MV)  st = 0;
            else if (mv >= s_batt_estop_mv + BATTERY_RECOVER_MV) st = 1;
        }

        switch (st) {
        case 2:
            safety_estop_assert(ESTOP_SRC_BATTERY_LOW);
            safety_caution_set(CAUTION_SRC_BATTERY_LOW, CAUTION_LEVEL_CRITICAL);
            break;
        case 1:
            safety_estop_clear_autoclearing(ESTOP_SRC_BATTERY_LOW);
            safety_caution_set(CAUTION_SRC_BATTERY_LOW, CAUTION_LEVEL_CAUTION);
            break;
        default:
            safety_estop_clear_autoclearing(ESTOP_SRC_BATTERY_LOW);
            safety_caution_clear(CAUTION_SRC_BATTERY_LOW);
            break;
        }

        if (st != s_batt_state) {
            if (st == 0)      log_record(LOG_MOD_BATTERY, LOG_SEV_INFO,     LOG_CODE_BATTERY_RESTORED, mv);
            else if (st == 1) log_record(LOG_MOD_BATTERY, LOG_SEV_WARN,     LOG_CODE_BATTERY_LOW,      mv);
            else              log_record(LOG_MOD_BATTERY, LOG_SEV_CRITICAL, LOG_CODE_BATTERY_ESTOP,    mv);
            s_batt_state = st;
        }
    }

    if (battery_present(BATTERY_6S)) {
        uint16_t mv6 = battery_mv(BATTERY_6S);
        if (!s_batt6_low && mv6 < BATTERY_6S_WARN_MV) {
            log_record(LOG_MOD_BATTERY, LOG_SEV_WARN, LOG_CODE_BATTERY_6S_LOW, mv6);
            s_batt6_low = true;
        } else if (s_batt6_low && mv6 >= BATTERY_6S_WARN_MV + BATTERY_RECOVER_MV) {
            s_batt6_low = false;
        }
    }
#endif
}

void safety_monitors_tick(void) {
    cargo_tick();
    current_tick();
    tof_tick_monitor();
    lidar_tick_monitor();
    battery_tick_monitor();
}

/* ===========================================================================
 *  Init
 * =========================================================================== */

void safety_init(void) {
    s_mode = MODE_SUPERVISED;
    s_func = FUNC_STANDBY;
    s_estop = 0;
    for (uint32_t i = 0; i < CAUTION_MAX_SRC; i++) s_level[i] = CAUTION_NORMAL;
    s_ws_override = 1.0f;
    s_ws_override_active = false;
    s_hb_last_ms = mcu_now_ms();
    s_hb_stage = 0;
    s_hb_unsupervised_due_to_timeout = false;
    s_oc_streak[0] = 0;
    s_oc_streak[1] = 0;
    s_tof_band = 0;
    s_lidar_band = 0;
    s_batt_state = 0;
    s_batt6_low = false;
    update_unsupervised_baseline();
}
