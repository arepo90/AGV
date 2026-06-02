#include "imu.h"
#include "config.h"
#include "i2c.h"
#include "log.h"
#include "mcu.h"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- BNO055 register subset --------------------------------------------- */
#define REG_CHIP_ID      0x00
#define REG_PAGE_ID      0x07
#define REG_EUL_HEADING  0x1A    /* 6 bytes: heading, roll, pitch (i16, 1/16 deg) */
#define REG_GYR_DATA     0x14    /* 6 bytes: X, Y, Z (i16, 1/16 dps) */
#define REG_UNIT_SEL     0x3B
#define REG_OPR_MODE     0x3D
#define REG_CALIB_STAT   0x35

#define OPMODE_CONFIG    0x00
#define CHIP_ID_VALUE    0xA0

static bool     s_present  = false;
static bool     s_have_data = false;
static int16_t  s_eul[3];        /* heading, roll, pitch (1/16 deg) */
static int16_t  s_gyr_z = 0;     /* 1/16 dps */
static uint8_t  s_calib = 0;
static uint32_t s_last_ms = 0;

static int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool bno055_setup(void) {
    uint8_t id = 0;
    if (!i2c_read_regs(BNO055_I2C_ADDR, REG_CHIP_ID, &id, 1)) return false;
    if (id != CHIP_ID_VALUE) {
        log_record(LOG_MOD_IMU, LOG_SEV_ERROR, LOG_CODE_IMU_I2C_NACK, id);
        return false;
    }
    if (!i2c_write_reg(BNO055_I2C_ADDR, REG_PAGE_ID, 0))             return false;
    if (!i2c_write_reg(BNO055_I2C_ADDR, REG_OPR_MODE, OPMODE_CONFIG)) return false;
    mcu_delay_ms(20);
    /* UNIT_SEL = 0: Windows orientation, deg, dps, m/s². */
    if (!i2c_write_reg(BNO055_I2C_ADDR, REG_UNIT_SEL, 0x00))         return false;
    if (!i2c_write_reg(BNO055_I2C_ADDR, REG_OPR_MODE, BNO055_OPMODE_NDOF)) return false;
    mcu_delay_ms(25);   /* CONFIG → NDOF ≥ 19 ms */
    return true;
}

void imu_init(void) {
#if DISABLE_IMU
    return;
#else
    i2c_init();
    s_present = bno055_setup();   /* logged inside on failure; we continue regardless */
#endif
}

void imu_tick(uint32_t now_ms) {
#if DISABLE_IMU
    (void)now_ms;
    return;
#else
    if (!s_present) return;
    if ((now_ms - s_last_ms) < (1000u / IMU_READ_HZ)) return;
    s_last_ms = now_ms;

    uint8_t buf[6];
    if (i2c_read_regs(BNO055_I2C_ADDR, REG_EUL_HEADING, buf, 6)) {
        s_eul[0] = le16(&buf[0]);
        s_eul[1] = le16(&buf[2]);
        s_eul[2] = le16(&buf[4]);
        s_have_data = true;
    }
    if (i2c_read_regs(BNO055_I2C_ADDR, REG_GYR_DATA, buf, 6)) {
        s_gyr_z = le16(&buf[4]);
    }
    uint8_t cal;
    if (i2c_read_regs(BNO055_I2C_ADDR, REG_CALIB_STAT, &cal, 1)) {
        if ((cal & 0xC0) != (s_calib & 0xC0)) {
            log_record(LOG_MOD_IMU, LOG_SEV_INFO,
                       (cal < s_calib) ? LOG_CODE_IMU_CALIB_LOST
                                       : LOG_CODE_IMU_CALIB_GAINED,
                       cal);
        }
        s_calib = cal;
    }
#endif
}

bool imu_present(void)  { return s_present; }
bool imu_has_data(void) { return s_have_data; }

float imu_yaw_deg(void)   { return (float)s_eul[0] / 16.0f; }
float imu_pitch_deg(void) { return (float)s_eul[2] / 16.0f; }
float imu_roll_deg(void)  { return (float)s_eul[1] / 16.0f; }

float imu_yaw_rad(void) {
    float y = imu_yaw_deg();
    if (y > 180.0f) y -= 360.0f;   /* 0..360 → [-180,180] */
    return IMU_HEADING_SIGN * y * (float)M_PI / 180.0f;
}

float imu_gyro_z_radps(void) {
    float dps = (float)s_gyr_z / 16.0f;
    return IMU_HEADING_SIGN * dps * (float)M_PI / 180.0f;
}

uint8_t imu_calib_sys(void)   { return (s_calib >> 6) & 0x3u; }
uint8_t imu_calib_gyro(void)  { return (s_calib >> 4) & 0x3u; }
uint8_t imu_calib_accel(void) { return (s_calib >> 2) & 0x3u; }
uint8_t imu_calib_mag(void)   { return  s_calib       & 0x3u; }
