#include "nav_line.h"
#include "analog.h"
#include "config.h"
#include "crc.h"
#include "flash.h"
#include "log.h"
#include "pid.h"
#include "types.h"
#include <stddef.h>
#include <string.h>

static uint16_t s_white[ANALOG_QTR_COUNT];
static uint16_t s_black[ANALOG_QTR_COUNT];
static pid_t    s_pid;
static bool     s_lost = false;
static float    s_position = 0.0f;

static float    s_cruise_mps     = LINE_FOLLOW_CRUISE_MPS;
static float    s_lost_threshold = QTR_LINE_LOST_THRESHOLD;

/* ---- Sweep-calibration state -------------------------------------------- */
static bool     s_cal_active = false;
static uint16_t s_cal_min[ANALOG_QTR_COUNT];
static uint16_t s_cal_max[ANALOG_QTR_COUNT];

/* ---- Flash schema ------------------------------------------------------- */
#define QTR_FLASH_MAGIC    0xC4118A33u
#define QTR_FLASH_VERSION  0x00000001u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint16_t white[ANALOG_QTR_COUNT];
    uint16_t black[ANALOG_QTR_COUNT];
    uint16_t crc16;
} qtr_record_t;

static uint16_t record_crc(const qtr_record_t *r) {
    return crc16_ccitt((const uint8_t *)r, offsetof(qtr_record_t, crc16));
}

void nav_line_init(void) {
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        s_white[i] = QTR_DEFAULT_WHITE;
        s_black[i] = QTR_DEFAULT_BLACK;
    }
    pid_init(&s_pid, LINE_FOLLOW_KP, LINE_FOLLOW_KI, LINE_FOLLOW_KD,
             -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS);
    s_lost = false;
    s_position = 0.0f;
}

void nav_line_reset(void) {
    pid_reset(&s_pid);
    s_lost = false;
}

/* ---- Sweep calibration --------------------------------------------------- */

void nav_line_cal_begin(void) {
    s_cal_active = true;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        s_cal_min[i] = 0xFFFFu;
        s_cal_max[i] = 0;
    }
    log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_QTR_CAL_BEGIN, 0);
}

void nav_line_cal_track(void) {
    if (!s_cal_active || !analog_has_data()) return;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        uint16_t raw = analog_qtr(i);
        if (raw < s_cal_min[i]) s_cal_min[i] = raw;
        if (raw > s_cal_max[i]) s_cal_max[i] = raw;
    }
}

void nav_line_cal_cancel(void) {
    if (!s_cal_active) return;
    s_cal_active = false;
    log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_QTR_CAL_CANCELED, 0);
}

bool nav_line_cal_active(void) { return s_cal_active; }

bool nav_line_cal_save(void) {
    if (!s_cal_active) return false;
    s_cal_active = false;

    uint8_t insufficient = 0;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        uint16_t mn = s_cal_min[i], mx = s_cal_max[i];
        if ((uint32_t)mx - (uint32_t)mn < 200u) insufficient |= (uint8_t)(1u << i);
        else { s_white[i] = mn; s_black[i] = mx; }
    }
    if (insufficient)
        log_record(LOG_MOD_NAV, LOG_SEV_WARN, LOG_CODE_QTR_CAL_INSUFFICIENT_RANGE, insufficient);

    qtr_record_t rec;
    rec.magic   = QTR_FLASH_MAGIC;
    rec.version = QTR_FLASH_VERSION;
    memcpy(rec.white, s_white, sizeof rec.white);
    memcpy(rec.black, s_black, sizeof rec.black);
    rec.crc16 = record_crc(&rec);

    bool ok = flash_write_page(&rec, sizeof rec);
    log_record(LOG_MOD_NAV, ok ? LOG_SEV_INFO : LOG_SEV_ERROR,
               ok ? LOG_CODE_QTR_CAL_END : LOG_CODE_FLASH_WRITE_FAIL, ok ? 1u : 0u);
    return ok;
}

bool nav_line_cal_reset_defaults(void) {
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        s_white[i] = QTR_DEFAULT_WHITE;
        s_black[i] = QTR_DEFAULT_BLACK;
    }
    s_cal_active = false;
    return flash_erase_page();
}

bool nav_line_load_calibration_from_flash(void) {
    qtr_record_t rec;
    flash_read_page(&rec, sizeof rec);

    if (rec.magic != QTR_FLASH_MAGIC) return false;   /* none yet / erased — not an error */
    if (rec.version != QTR_FLASH_VERSION) {
        log_record(LOG_MOD_NAV, LOG_SEV_WARN, LOG_CODE_FLASH_LOAD_FAIL, rec.version);
        return false;
    }
    if (rec.crc16 != record_crc(&rec)) {
        log_record(LOG_MOD_NAV, LOG_SEV_ERROR, LOG_CODE_FLASH_LOAD_FAIL, rec.crc16);
        return false;
    }
    memcpy(s_white, rec.white, sizeof s_white);
    memcpy(s_black, rec.black, sizeof s_black);
    log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_FLASH_LOAD_OK, 0);
    return true;
}

/* ---- Control law --------------------------------------------------------- */

bool  nav_line_is_lost(void)  { return s_lost; }
float nav_line_position(void) { return s_position; }

void nav_line_get(float dt_s, float *v_target, float *omega_target) {
    if (!analog_has_data()) { *v_target = 0.0f; *omega_target = 0.0f; return; }

    float weighted = 0.0f, total = 0.0f;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        int32_t range = (int32_t)s_black[i] - (int32_t)s_white[i];
        if (range <= 0) continue;   /* misconfigured sensor — skip */
        float n = (float)((int32_t)analog_qtr(i) - (int32_t)s_white[i]) / (float)range;
        if (n < 0.0f) n = 0.0f;
        if (n > 1.0f) n = 1.0f;
#if QTR_INVERT_ARRAY
        float idx = (float)((ANALOG_QTR_COUNT - 1u) - i);
#else
        float idx = (float)i;
#endif
        weighted += n * idx;
        total    += n;
    }

    if (total < s_lost_threshold) {
        if (!s_lost) {
            log_record(LOG_MOD_NAV, LOG_SEV_WARN, LOG_CODE_LINE_LOST, 0);
            s_lost = true;
            pid_reset(&s_pid);
        }
        *v_target = 0.0f; *omega_target = 0.0f;
        return;
    }
    s_lost = false;

    float centre   = ((float)ANALOG_QTR_COUNT - 1.0f) * 0.5f;
    float centroid = weighted / total;
    s_position = (centroid - centre) / centre;     /* [-1, +1] */

    /* Setpoint 0 (line centred). error = 0 - position; Kp>0 turns toward it. */
    float omega = pid_step(&s_pid, 0.0f, s_position, 0.0f, dt_s);

    float v = s_cruise_mps;
    if (s_position > 0.6f || s_position < -0.6f) v *= 0.5f;   /* slow on sharp corrections */

    *v_target = v;
    *omega_target = omega;
}

void nav_line_set_cruise_mps(float v)    { if (v >= 0.0f && v <= 5.0f) s_cruise_mps = v; }
void nav_line_set_lost_threshold(float t){ if (t > 0.0f && t < 8.0f) s_lost_threshold = t; }
void nav_line_set_gains(float kp, float ki, float kd) { pid_set_gains(&s_pid, kp, ki, kd); }
