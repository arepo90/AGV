#include "imu.h"
#include "config.h"
#include "i2c.h"
#include "log.h"
#include "mcu.h"
#include "types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- MPU6050 register subset -------------------------------------------- */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A    /* DLPF_CFG in bits [2:0] */
#define REG_GYRO_CONFIG  0x1B    /* FS_SEL in bits [4:3]   */
#define REG_ACCEL_CONFIG 0x1C    /* AFS_SEL in bits [4:3]  */
#define REG_ACCEL_XOUT_H 0x3B    /* 14 bytes: AX,AY,AZ,TEMP,GX,GY,GZ (i16 BE) */
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define PWR_RESET        0x80
#define PWR_CLKSEL_PLLX  0x01    /* wake + PLL w/ X-gyro ref (more stable than RC) */
#define WHO_AM_I_VALUE   0x68

/* "at rest" gates for ZUPT: accel within this of 1 g, gyro below this rate. */
#define STILL_ACCEL_TOL_G   0.06f
#define STILL_GYRO_DPS      1.5f

static bool     s_present   = false;
static bool     s_have_data = false;
static bool     s_still     = false;
static float    s_gyro_z_radps = 0.0f;
static float    s_pitch_deg = 0.0f, s_roll_deg = 0.0f;
static uint32_t s_last_ms   = 0;

/* MPU6050 burst data is big-endian (high byte first). */
static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static bool mpu6050_setup(void) {
    uint8_t id = 0;
    if (!i2c_read_regs(MPU6050_I2C_ADDR, REG_WHO_AM_I, &id, 1)) {
        log_record(LOG_MOD_IMU, LOG_SEV_ERROR, LOG_CODE_IMU_I2C_NACK, 0);
        return false;
    }
    if (id != WHO_AM_I_VALUE) {
        log_record(LOG_MOD_IMU, LOG_SEV_ERROR, LOG_CODE_IMU_I2C_NACK, id);
        return false;
    }
    /* Reset, let it settle, then wake on the gyro PLL clock. */
    if (!i2c_write_reg(MPU6050_I2C_ADDR, REG_PWR_MGMT_1, PWR_RESET)) return false;
    mcu_delay_ms(100);
    if (!i2c_write_reg(MPU6050_I2C_ADDR, REG_PWR_MGMT_1, PWR_CLKSEL_PLLX)) return false;
    mcu_delay_ms(10);
    /* DLPF tames motor/enclosure vibration; sample rate, then full-scale ranges. */
    if (!i2c_write_reg(MPU6050_I2C_ADDR, REG_CONFIG,       MPU6050_DLPF_CFG))        return false;
    if (!i2c_write_reg(MPU6050_I2C_ADDR, REG_SMPLRT_DIV,   MPU6050_SMPLRT_DIV))      return false;
    if (!i2c_write_reg(MPU6050_I2C_ADDR, REG_GYRO_CONFIG,  MPU6050_GYRO_FS_SEL << 3))  return false;
    if (!i2c_write_reg(MPU6050_I2C_ADDR, REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL << 3)) return false;
    return true;
}

void imu_init(void) {
#if DISABLE_IMU
    return;
#else
    i2c_init();
    s_present = mpu6050_setup();   /* logged inside on failure; we continue regardless */
#endif
}

void imu_tick(uint32_t now_ms) {
#if DISABLE_IMU
    (void)now_ms;
    return;
#else
    if (!s_present) return;
    if ((now_ms - s_last_ms) < (1000u / IMU_READ_HZ)) return;
    float dt = (s_last_ms == 0) ? 0.0f : (float)(now_ms - s_last_ms) * 0.001f;
    s_last_ms = now_ms;

    uint8_t buf[14];
    if (!i2c_read_regs(MPU6050_I2C_ADDR, REG_ACCEL_XOUT_H, buf, 14)) {
        log_record(LOG_MOD_IMU, LOG_SEV_WARN, LOG_CODE_IMU_I2C_TIMEOUT, 0);
        return;
    }

    /* Accel (g) and gyro (deg/s); skip bytes 6-7 (temperature). */
    float ax = (float)be16(&buf[0])  / MPU6050_ACCEL_LSB_PER_G;
    float ay = (float)be16(&buf[2])  / MPU6050_ACCEL_LSB_PER_G;
    float az = (float)be16(&buf[4])  / MPU6050_ACCEL_LSB_PER_G;
    float gx_dps = (float)be16(&buf[8])  / MPU6050_GYRO_LSB_PER_DPS;
    float gy_dps = (float)be16(&buf[10]) / MPU6050_GYRO_LSB_PER_DPS;
    float gz_dps = (float)be16(&buf[12]) / MPU6050_GYRO_LSB_PER_DPS;

    s_gyro_z_radps = IMU_HEADING_SIGN * gz_dps * (float)M_PI / 180.0f;

    /* Complementary tilt: integrate gyro, slow-correct toward the gravity vector. */
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_mag > 0.1f && dt > 0.0f) {
        float roll_acc  = atan2f(ay, az) * 180.0f / (float)M_PI;
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
        float roll_g  = s_roll_deg  + gx_dps * dt;
        float pitch_g = s_pitch_deg + gy_dps * dt;
        s_roll_deg  = IMU_TILT_COMP_ALPHA * roll_g  + (1.0f - IMU_TILT_COMP_ALPHA) * roll_acc;
        s_pitch_deg = IMU_TILT_COMP_ALPHA * pitch_g + (1.0f - IMU_TILT_COMP_ALPHA) * pitch_acc;
    }

    /* "Still" if the chassis is upright-ish at 1 g and barely rotating — used to
     * gate the zero-velocity bias update in odometry. */
    float gyro_mag_dps = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);
    s_still = (fabsf(accel_mag - 1.0f) < STILL_ACCEL_TOL_G) &&
              (gyro_mag_dps < STILL_GYRO_DPS);

    s_have_data = true;
#endif
}

bool imu_present(void)  { return s_present; }
bool imu_has_data(void) { return s_have_data; }
bool imu_is_still(void) { return s_still; }

float imu_gyro_z_radps(void) { return s_gyro_z_radps; }
float imu_pitch_deg(void)    { return s_pitch_deg; }
float imu_roll_deg(void)     { return s_roll_deg; }
