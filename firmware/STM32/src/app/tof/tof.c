#include "tof.h"
#include "config.h"
#include "i2c.h"
#include "log.h"
#include "mcu.h"
#include "types.h"

/* =============================================================================
 *  VL53L0X minimal driver (single-shot init → continuous ranging). Register set
 *  and init order follow the ST API / Pololu minimal port, rewritten over the
 *  hal/i2c helpers. The measurement timing budget is left at the post-init
 *  default (~33 ms, ~1.2 m range) — ample for corner obstacle bands.
 * =============================================================================
 */

/* ---- VL53L0X register subset -------------------------------------------- */
#define REG_SYSRANGE_START                   0x00
#define REG_SYSTEM_SEQUENCE_CONFIG           0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO     0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR           0x0B
#define REG_RESULT_INTERRUPT_STATUS          0x13
#define REG_RESULT_RANGE_STATUS              0x14   /* range mm at +10 (0x1E) */
#define REG_MSRC_CONFIG_CONTROL              0x60
#define REG_FINAL_RANGE_MIN_COUNT_RATE       0x44
#define REG_GPIO_HV_MUX_ACTIVE_HIGH          0x84
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define REG_DYNAMIC_SPAD_NUM_REQUESTED       0x4E
#define REG_GLOBAL_CONFIG_REF_EN_START_SEL   0xB6
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define REG_MODEL_ID                         0xC0
#define MODEL_ID_VALUE                       0xEE

/* Static-init tuning table (reg, val pairs) — opaque ST-supplied magic. */
static const uint8_t TUNING[] = {
    0xFF,0x01, 0x00,0x00,
    0xFF,0x00, 0x09,0x00, 0x10,0x00, 0x11,0x00,
    0x24,0x01, 0x25,0xFF, 0x75,0x00,
    0xFF,0x01, 0x4E,0x2C, 0x48,0x00, 0x30,0x20,
    0xFF,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04, 0x32,0x03, 0x40,0x83,
    0x46,0x25, 0x60,0x00, 0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
    0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00, 0x64,0x00, 0x65,0x00, 0x66,0xA0,
    0xFF,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xFF, 0x4A,0x00,
    0xFF,0x00, 0x7A,0x0A, 0x7B,0x00, 0x78,0x21,
    0xFF,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xFF, 0x45,0x26, 0x46,0x05,
    0x40,0x40, 0x0E,0x06, 0x20,0x1A, 0x43,0x40,
    0xFF,0x00, 0x34,0x03, 0x35,0x44,
    0xFF,0x01, 0x31,0x04, 0x4B,0x09, 0x4C,0x05, 0x4D,0x04,
    0xFF,0x00, 0x44,0x00, 0x45,0x20, 0x47,0x08, 0x48,0x28, 0x67,0x00,
    0x70,0x04, 0x71,0x01, 0x72,0xFE, 0x76,0x00, 0x77,0x00,
    0xFF,0x01, 0x0D,0x01,
    0xFF,0x00, 0x80,0x01, 0x01,0xF8,
    0xFF,0x01, 0x8E,0x01, 0x00,0x01,
    0xFF,0x00, 0x80,0x00,
};

static const uint8_t MUX_CH[TOF_NUM_SENSORS] = {
    TOF_MUX_CH_FRONT, TOF_MUX_CH_REAR, TOF_MUX_CH_LEFT, TOF_MUX_CH_RIGHT,
};

static bool     s_present[TOF_NUM_SENSORS];
static uint16_t s_dist_mm[TOF_NUM_SENSORS];
static uint8_t  s_stop_var[TOF_NUM_SENSORS];
static uint32_t s_idx = 0;
static uint32_t s_last_ms = 0;

/* ---- register helpers (caller has already selected the mux channel) ----- */
static bool w8(uint8_t reg, uint8_t val)  { return i2c_write_reg(VL53L0X_I2C_ADDR, reg, val); }
static bool r8(uint8_t reg, uint8_t *val) { return i2c_read_regs(VL53L0X_I2C_ADDR, reg, val, 1); }

static bool w16(uint8_t reg, uint16_t val) {
    uint8_t b[2] = { (uint8_t)(val >> 8), (uint8_t)val };
    return i2c_write_regs(VL53L0X_I2C_ADDR, reg, b, 2);
}
static bool r16(uint8_t reg, uint16_t *val) {
    uint8_t b[2];
    if (!i2c_read_regs(VL53L0X_I2C_ADDR, reg, b, 2)) return false;
    *val = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
    return true;
}

/* Expose exactly one mux downstream channel (BNO055/INA219 stay upstream). */
static bool mux_select(uint8_t channel) {
    uint8_t mask = (uint8_t)(1u << channel);
    return i2c_write(TCA9548A_I2C_ADDR, &mask, 1);
}

/* VL53L0X_perform_single_ref_calibration() */
static bool ref_calibration(uint8_t vhv_init_byte) {
    if (!w8(REG_SYSRANGE_START, (uint8_t)(0x01u | vhv_init_byte))) return false;
    uint32_t deadline = mcu_now_ms() + 100u;
    uint8_t status;
    do {
        if (!r8(REG_RESULT_INTERRUPT_STATUS, &status)) return false;
        if (status & 0x07u) break;
    } while ((int32_t)(mcu_now_ms() - deadline) < 0);
    if (!(status & 0x07u)) return false;
    return w8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) && w8(REG_SYSRANGE_START, 0x00);
}

/* VL53L0X_get_spad_info_from_device() */
static bool get_spad_info(uint8_t *count, bool *is_aperture) {
    uint8_t v;
    bool ok = w8(0x80, 0x01) && w8(0xFF, 0x01) && w8(0x00, 0x00) && w8(0xFF, 0x06)
           && r8(0x83, &v) && w8(0x83, (uint8_t)(v | 0x04))
           && w8(0xFF, 0x07) && w8(0x81, 0x01) && w8(0x80, 0x01)
           && w8(0x94, 0x6B) && w8(0x83, 0x00);
    if (!ok) return false;

    uint32_t deadline = mcu_now_ms() + 100u;
    do {
        if (!r8(0x83, &v)) return false;
        if (v != 0x00) break;
    } while ((int32_t)(mcu_now_ms() - deadline) < 0);
    if (v == 0x00) return false;

    uint8_t tmp;
    ok = w8(0x83, 0x01) && r8(0x92, &tmp)
      && w8(0x81, 0x00) && w8(0xFF, 0x06)
      && r8(0x83, &v) && w8(0x83, (uint8_t)(v & ~0x04u))
      && w8(0xFF, 0x01) && w8(0x00, 0x01) && w8(0xFF, 0x00) && w8(0x80, 0x00);
    if (!ok) return false;

    *count = tmp & 0x7Fu;
    *is_aperture = (tmp >> 7) & 0x01u;
    return true;
}

/* VL53L0X_DataInit + StaticInit + ref calibration. Returns false on any NACK. */
static bool sensor_init(uint32_t i) {
    uint8_t v;
    if (!r8(REG_MODEL_ID, &v) || v != MODEL_ID_VALUE) return false;

    /* --- DataInit --- */
    bool ok = r8(0x89, &v) && w8(0x89, (uint8_t)(v | 0x01))   /* 2V8 I/O */
           && w8(0x88, 0x00)                                   /* I2C standard */
           && w8(0x80, 0x01) && w8(0xFF, 0x01) && w8(0x00, 0x00)
           && r8(0x91, &s_stop_var[i])
           && w8(0x00, 0x01) && w8(0xFF, 0x00) && w8(0x80, 0x00)
           && r8(REG_MSRC_CONFIG_CONTROL, &v)
           && w8(REG_MSRC_CONFIG_CONTROL, (uint8_t)(v | 0x12))
           && w16(REG_FINAL_RANGE_MIN_COUNT_RATE, 32u)         /* 0.25 MCPS */
           && w8(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    if (!ok) return false;

    /* --- StaticInit: reference SPAD map --- */
    uint8_t spad_count; bool aperture;
    if (!get_spad_info(&spad_count, &aperture)) return false;

    uint8_t spad_map[6];
    if (!i2c_read_regs(VL53L0X_I2C_ADDR, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6))
        return false;

    ok = w8(0xFF, 0x01) && w8(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
      && w8(REG_DYNAMIC_SPAD_NUM_REQUESTED, 0x2C) && w8(0xFF, 0x00)
      && w8(REG_GLOBAL_CONFIG_REF_EN_START_SEL, 0xB4);
    if (!ok) return false;

    uint8_t first = aperture ? 12u : 0u;
    uint8_t enabled = 0;
    for (uint8_t s = 0; s < 48u; s++) {
        if (s < first || enabled == spad_count)
            spad_map[s / 8] &= (uint8_t)~(1u << (s % 8));
        else if ((spad_map[s / 8] >> (s % 8)) & 0x01u)
            enabled++;
    }
    if (!i2c_write_regs(VL53L0X_I2C_ADDR, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6))
        return false;

    /* --- StaticInit: tuning table + interrupt config --- */
    for (uint32_t t = 0; t < sizeof TUNING; t += 2)
        if (!w8(TUNING[t], TUNING[t + 1])) return false;

    ok = w8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
      && r8(REG_GPIO_HV_MUX_ACTIVE_HIGH, &v)
      && w8(REG_GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)(v & ~0x10u))   /* int active low */
      && w8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
      && w8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    if (!ok) return false;

    /* --- Reference calibration (VHV + phase) --- */
    if (!w8(REG_SYSTEM_SEQUENCE_CONFIG, 0x01) || !ref_calibration(0x40)) return false;
    if (!w8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02) || !ref_calibration(0x00)) return false;
    return w8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
}

/* VL53L0X_StartMeasurement() — continuous back-to-back. */
static bool start_continuous(uint32_t i) {
    return w8(0x80, 0x01) && w8(0xFF, 0x01) && w8(0x00, 0x00)
        && w8(0x91, s_stop_var[i]) && w8(0x00, 0x01) && w8(0xFF, 0x00)
        && w8(0x80, 0x00) && w8(REG_SYSRANGE_START, 0x02);
}

/* Non-waiting: read the latest result only if the sensor flagged one ready. */
static bool read_if_ready(uint16_t *mm) {
    uint8_t status;
    if (!r8(REG_RESULT_INTERRUPT_STATUS, &status)) return false;
    if ((status & 0x07u) == 0) return false;          /* still converting */
    uint16_t range;
    bool ok = r16((uint8_t)(REG_RESULT_RANGE_STATUS + 10), &range);
    w8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);             /* arm the next measurement */
    if (!ok) return false;
    *mm = range;
    return true;
}

void tof_init(void) {
#if DISABLE_TOF
    return;
#else
    i2c_init();
    for (uint32_t i = 0; i < TOF_NUM_SENSORS; i++) {
        s_dist_mm[i] = TOF_VALID_MAX_MM;
        s_present[i] = false;
        if (!mux_select(MUX_CH[i])) continue;
        mcu_delay_ms(2);                  /* post-power boot settle */
        if (sensor_init(i) && start_continuous(i)) {
            s_present[i] = true;
        } else {
            log_record(LOG_MOD_TOF, LOG_SEV_WARN, LOG_CODE_TOF_INIT_FAIL, MUX_CH[i]);
        }
    }
    s_last_ms = mcu_now_ms();
#endif
}

void tof_tick(uint32_t now_ms) {
#if DISABLE_TOF
    (void)now_ms;
    return;
#else
    if ((now_ms - s_last_ms) < (1000u / TOF_POLL_HZ)) return;
    s_last_ms = now_ms;

    uint32_t i = s_idx;
    s_idx = (s_idx + 1u) % TOF_NUM_SENSORS;   /* one sensor per pass */
    if (!s_present[i]) return;

    if (!mux_select(MUX_CH[i])) {
        log_record(LOG_MOD_TOF, LOG_SEV_WARN, LOG_CODE_TOF_I2C_FAIL, MUX_CH[i]);
        return;
    }
    uint16_t mm;
    if (!read_if_ready(&mm)) return;          /* not ready or read error: keep last */
    /* 0 or out-of-range → treat as "clear" so a flaky read never false-trips. */
    s_dist_mm[i] = (mm == 0 || mm >= TOF_VALID_MAX_MM) ? TOF_VALID_MAX_MM : mm;
#endif
}

bool     tof_present(uint32_t i)     { return (i < TOF_NUM_SENSORS) && s_present[i]; }
uint16_t tof_distance_mm(uint32_t i) { return (i < TOF_NUM_SENSORS) ? s_dist_mm[i] : TOF_VALID_MAX_MM; }

bool tof_any_present(void) {
    for (uint32_t i = 0; i < TOF_NUM_SENSORS; i++) if (s_present[i]) return true;
    return false;
}

uint16_t tof_min_distance_mm(void) {
    uint16_t m = TOF_VALID_MAX_MM;
    for (uint32_t i = 0; i < TOF_NUM_SENSORS; i++)
        if (s_present[i] && s_dist_mm[i] < m) m = s_dist_mm[i];
    return m;
}
