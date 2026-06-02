#include "i2c.h"
#include "stm32f0xx.h"

/* TIMINGR for 400 kHz @ 48 MHz APB1 (RM0091 example): PRESC=5, SCLL=9, SCLH=3,
 * SDADEL=3, SCLDEL=3. */
#define I2C_TIMINGR_400KHZ   0x00B01A4Bu
#define I2C_TIMEOUT_CYCLES   20000u       /* ~5 ms at 48 MHz */

void i2c_init(void) {
    /* Idempotent: the IMU, TOF and battery modules each bring up the shared bus
     * independently and any subset may be DISABLE_*'d, so guard re-entry. */
    static bool s_inited = false;
    if (s_inited) return;
    s_inited = true;

    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB8 SCL, PB9 SDA → AF1, open-drain, no internal pulls (external 4.7 kΩ). */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))
                 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFu << 0) | (0xFu << 4)))
                  | (1u << 0) | (1u << 4);

    I2C1->CR1     = 0;
    I2C1->TIMINGR = I2C_TIMINGR_400KHZ;
    I2C1->CR1     = I2C_CR1_PE;
}

/* Spin until `flag` is set; bail on NACK / bus error / arbitration loss. */
static bool wait_flag(uint32_t flag) {
    for (uint32_t i = 0; i < I2C_TIMEOUT_CYCLES; i++) {
        uint32_t isr = I2C1->ISR;
        if (isr & flag) return true;
        if (isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) return false;
    }
    return false;
}

/* Single START → write `n` bytes from `buf` → AUTOEND STOP. */
bool i2c_write(uint8_t addr7, const uint8_t *buf, uint8_t n) {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint8_t i = 0; i < n; i++) {
        if (!wait_flag(I2C_ISR_TXIS)) return false;
        I2C1->TXDR = buf[i];
    }
    if (!wait_flag(I2C_ISR_STOPF)) return false;
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

/* Write a register pointer followed by `n` payload bytes (single transaction). */
bool i2c_write_regs(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint8_t n) {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)(n + 1u) << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START | I2C_CR2_AUTOEND;
    if (!wait_flag(I2C_ISR_TXIS)) return false;
    I2C1->TXDR = reg;
    for (uint8_t i = 0; i < n; i++) {
        if (!wait_flag(I2C_ISR_TXIS)) return false;
        I2C1->TXDR = buf[i];
    }
    if (!wait_flag(I2C_ISR_STOPF)) return false;
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

bool i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val) {
    return i2c_write_regs(addr7, reg, &val, 1);
}

bool i2c_read_regs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n) {
    /* Phase 1: write the register pointer (no STOP — manual RESTART follows). */
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | (1u << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START;
    if (!wait_flag(I2C_ISR_TXIS)) return false;
    I2C1->TXDR = reg;
    if (!wait_flag(I2C_ISR_TC)) return false;

    /* Phase 2: repeated start, read n bytes, auto STOP. */
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint8_t i = 0; i < n; i++) {
        if (!wait_flag(I2C_ISR_RXNE)) return false;
        buf[i] = (uint8_t)I2C1->RXDR;
    }
    if (!wait_flag(I2C_ISR_STOPF)) return false;
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}
