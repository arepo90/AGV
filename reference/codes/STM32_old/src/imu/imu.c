#include "imu.h"
#include "config.h"
#include "log.h"
#include "stm32f0xx.h"
#include "types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- BNO055 register map (subset) --------------------------------------- */
#define BNO055_REG_CHIP_ID       0x00
#define BNO055_REG_OPR_MODE      0x3D
#define BNO055_REG_PWR_MODE      0x3E
#define BNO055_REG_PAGE_ID       0x07
#define BNO055_REG_SYS_TRIGGER   0x3F
#define BNO055_REG_UNIT_SEL      0x3B
#define BNO055_REG_GYR_DATA      0x14    /* 6 bytes: gyro X, Y, Z (i16 each, 1/16 dps) */
#define BNO055_REG_EUL_HEADING   0x1A    /* 6 bytes: heading, roll, pitch (i16 each, 1/16 deg) */
#define BNO055_REG_CALIB_STAT    0x35

#define BNO055_OPMODE_CONFIG     0x00
#define BNO055_CHIP_ID_VALUE     0xA0

/* ---- I2C timing for 400 kHz @ 48 MHz APB --------------------------------
 * Per RM0091 §I2C example. PRESC=5, SCLDEL=3, SDADEL=3, SCLH=3, SCLL=9.
 * TIMINGR = 0x00B01A4B */
#define I2C_TIMINGR_400KHZ       0x00B01A4Bu

#define I2C_TIMEOUT_CYCLES       20000u    /* ~5 ms at 48 MHz when waiting on a flag */

/* ---- Module state ------------------------------------------------------- */
static bool    s_have_data = false;
static int16_t s_eul[3];       /* heading, roll, pitch (1/16 degree) */
static int16_t s_gyr_z = 0;    /* Z-axis gyro (1/16 dps) — used by EKF */
static uint8_t s_calib = 0;
static uint32_t s_last_ms = 0;

/* ---- Low-level I2C ----------------------------------------------------- */

static void i2c_init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB8 SCL, PB9 SDA → AF1, open-drain, no pull (using external 4.7 kΩ). */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))
                 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFu << 0) | (0xFu << 4)))
                  | (1u << 0) | (1u << 4);

    /* Reset, configure, enable. I2C clock source defaults to APB1. */
    I2C1->CR1     = 0;
    I2C1->TIMINGR = I2C_TIMINGR_400KHZ;
    I2C1->CR1     = I2C_CR1_PE;
}

/* Wait for ISR flag. Returns true if flag set within timeout. */
static bool i2c_wait(uint32_t flag) {
    for (uint32_t i = 0; i < I2C_TIMEOUT_CYCLES; i++) {
        uint32_t isr = I2C1->ISR;
        if (isr & flag) return true;
        if (isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) return false;
    }
    return false;
}

static bool i2c_write_then_read(uint8_t addr7, uint8_t reg, uint8_t *rx, uint8_t n) {
    /* ---- Write register address (1 byte) ---- */
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | (1u << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START;            /* AUTOEND=0; we'll do RESTART manually */
    if (!i2c_wait(I2C_ISR_TXIS))   { log_record(LOG_MOD_IMU, LOG_SEV_WARN,
                                                LOG_CODE_IMU_I2C_TIMEOUT, 1); return false; }
    I2C1->TXDR = reg;
    if (!i2c_wait(I2C_ISR_TC))     { log_record(LOG_MOD_IMU, LOG_SEV_WARN,
                                                LOG_CODE_IMU_I2C_NACK, 2); return false; }

    /* ---- Repeated start; read N bytes ---- */
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;

    for (uint8_t i = 0; i < n; i++) {
        if (!i2c_wait(I2C_ISR_RXNE)) { log_record(LOG_MOD_IMU, LOG_SEV_WARN,
                                                  LOG_CODE_IMU_I2C_TIMEOUT, 3); return false; }
        rx[i] = (uint8_t)I2C1->RXDR;
    }
    if (!i2c_wait(I2C_ISR_STOPF))   { log_record(LOG_MOD_IMU, LOG_SEV_WARN,
                                                 LOG_CODE_IMU_I2C_TIMEOUT, 4); return false; }
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

static bool i2c_write_byte(uint8_t addr7, uint8_t reg, uint8_t val) {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | (2u << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START | I2C_CR2_AUTOEND;
    if (!i2c_wait(I2C_ISR_TXIS)) return false;
    I2C1->TXDR = reg;
    if (!i2c_wait(I2C_ISR_TXIS)) return false;
    I2C1->TXDR = val;
    if (!i2c_wait(I2C_ISR_STOPF)) return false;
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

/* ---- Coarse delay (ms) without using SysTick to avoid coupling ---------- */
static void crude_delay_ms(uint32_t ms) {
    /* About 12 cycles per loop iteration at -Og, 48 MHz → 4 M iter/sec. */
    uint32_t loops = ms * 4000u;
    for (volatile uint32_t i = 0; i < loops; i++) { }
}

/* ---- BNO055 setup ------------------------------------------------------ */

static bool bno055_setup(void) {
    uint8_t id = 0;
    if (!i2c_write_then_read(BNO055_I2C_ADDR, BNO055_REG_CHIP_ID, &id, 1)) return false;
    if (id != BNO055_CHIP_ID_VALUE) {
        log_record(LOG_MOD_IMU, LOG_SEV_ERROR, LOG_CODE_IMU_I2C_NACK, id);
        return false;
    }

    /* Page 0, CONFIG mode, default units (deg, m/s², dps), then NDOF. */
    if (!i2c_write_byte(BNO055_I2C_ADDR, BNO055_REG_PAGE_ID, 0)) return false;
    if (!i2c_write_byte(BNO055_I2C_ADDR, BNO055_REG_OPR_MODE, BNO055_OPMODE_CONFIG)) return false;
    crude_delay_ms(20);

    /* UNIT_SEL: bit7=0 Windows orientation, bit4=0 °C, bit2=0 deg, bit1=0 dps, bit0=0 m/s². */
    if (!i2c_write_byte(BNO055_I2C_ADDR, BNO055_REG_UNIT_SEL, 0x00)) return false;
    if (!i2c_write_byte(BNO055_I2C_ADDR, BNO055_REG_OPR_MODE, BNO055_OPMODE_NDOF)) return false;
    crude_delay_ms(25);  /* CONFIG → NDOF transition ≥ 19 ms per datasheet */

    return true;
}

/* ---- Public API -------------------------------------------------------- */

void imu_init(void) {
#if DISABLE_IMU
    return;
#else
    i2c_init();
    if (!bno055_setup()) {
        /* Logged inside bno055_setup; firmware continues, imu_has_data() stays
         * false so the fusion code knows not to consume the readings. */
    }
#endif
}

static int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

void imu_tick(uint32_t now_ms) {
#if DISABLE_IMU
    (void)now_ms;
    return;
#else
    uint32_t period_ms = 1000u / IMU_READ_HZ;
    if ((now_ms - s_last_ms) < period_ms) return;
    s_last_ms = now_ms;

    uint8_t buf[6];
    if (i2c_write_then_read(BNO055_I2C_ADDR, BNO055_REG_EUL_HEADING, buf, 6)) {
        s_eul[0] = le16(&buf[0]);
        s_eul[1] = le16(&buf[2]);
        s_eul[2] = le16(&buf[4]);
        s_have_data = true;
    }

    /* Z-axis gyro for the EKF. Read separately; if it fails we just hold the
     * last value (the EKF inflates covariance in absence of fresh data). */
    if (i2c_write_then_read(BNO055_I2C_ADDR, BNO055_REG_GYR_DATA, buf, 6)) {
        s_gyr_z = le16(&buf[4]);  /* X@0, Y@2, Z@4 */
    }

    /* Calib status one byte. Cheap; do it every read so the workstation can
     * see calibration progress in real time. */
    uint8_t cal;
    if (i2c_write_then_read(BNO055_I2C_ADDR, BNO055_REG_CALIB_STAT, &cal, 1)) {
        /* Only log when the system-calibration field (top 2 bits) changes, so
         * the workstation can prompt the operator to do figure-eight motions
         * when it drops. */
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

float imu_yaw_deg(void)   { return (float)s_eul[0] / 16.0f; }
float imu_pitch_deg(void) { return (float)s_eul[2] / 16.0f; }
float imu_roll_deg(void)  { return (float)s_eul[1] / 16.0f; }

float imu_yaw_rad(void) {
    float y_deg = imu_yaw_deg();
    /* BNO055 returns 0..360 — wrap to [-π, +π], then apply sign to match the
     * encoder math convention (CCW-positive). */
    if (y_deg > 180.0f) y_deg -= 360.0f;
    return IMU_HEADING_SIGN * y_deg * (float)M_PI / 180.0f;
}

float imu_gyro_z_radps(void) {
    /* Raw gyro register is 1/16 dps per LSB (UNIT_SEL bit1=0 → dps). Same
     * sign convention as imu_yaw_rad(). */
    float dps = (float)s_gyr_z / 16.0f;
    return IMU_HEADING_SIGN * dps * (float)M_PI / 180.0f;
}

uint8_t imu_calib_sys(void)   { return (s_calib >> 6) & 0x3; }
uint8_t imu_calib_gyro(void)  { return (s_calib >> 4) & 0x3; }
uint8_t imu_calib_accel(void) { return (s_calib >> 2) & 0x3; }
uint8_t imu_calib_mag(void)   { return  s_calib       & 0x3; }

bool imu_has_data(void) { return s_have_data; }
