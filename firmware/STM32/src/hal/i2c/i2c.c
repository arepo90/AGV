#include "i2c.h"
#include "stm32f0xx.h"

/* TIMINGR for 100 kHz @ 48 MHz kernel clock (ST CubeMX value). Standard mode is
 * deliberate: forgiving of the long JST harness + 4.7 kΩ pulls, and the bus
 * only carries one INA219 at 2 Hz. */
#define I2C_TIMINGR_100KHZ   0x10805E89u
#define I2C_TIMEOUT_CYCLES   20000u       /* ~5 ms at 48 MHz */

void i2c_init(void) {
    /* Idempotent: every consumer of the shared bus brings it up independently
     * and any subset may be DISABLE_*'d, so guard re-entry. */
    static bool s_inited = false;
    if (s_inited) return;
    s_inited = true;

    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* I2C1 kernel clock = SYSCLK (48 MHz). The reset default is HSI (8 MHz),
     * which would silently scale the TIMINGR bus speed by 1/6. */
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

    /* PB8 SCL, PB9 SDA → AF1, open-drain, no internal pulls (external 4.7 kΩ). */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))
                 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFu << 0) | (0xFu << 4)))
                  | (1u << 0) | (1u << 4);

    I2C1->CR1     = 0;
    I2C1->TIMINGR = I2C_TIMINGR_100KHZ;
    I2C1->CR1     = I2C_CR1_PE;
}

/* Abandon a transfer that NACKed/hung: force a STOP, wait (bounded) for the
 * bus to release, and clear every sticky flag so the next transfer starts from
 * a known state. Without this, one wedged transaction poisons the bus until
 * reboot. */
static void bus_abort(void) {
    if (I2C1->ISR & I2C_ISR_BUSY) {
        I2C1->CR2 |= I2C_CR2_STOP;
        for (uint32_t i = 0; i < I2C_TIMEOUT_CYCLES; i++) {
            if (!(I2C1->ISR & I2C_ISR_BUSY)) break;
        }
    }
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
}

/* Wait for the bus to be free before issuing a START; abort + retry-once if a
 * previous failure left it claimed. */
static bool wait_idle(void) {
    for (uint32_t i = 0; i < I2C_TIMEOUT_CYCLES; i++) {
        if (!(I2C1->ISR & I2C_ISR_BUSY)) return true;
    }
    bus_abort();
    return !(I2C1->ISR & I2C_ISR_BUSY);
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
    if (!wait_idle()) return false;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint8_t i = 0; i < n; i++) {
        if (!wait_flag(I2C_ISR_TXIS)) { bus_abort(); return false; }
        I2C1->TXDR = buf[i];
    }
    if (!wait_flag(I2C_ISR_STOPF)) { bus_abort(); return false; }
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

/* Write a register pointer followed by `n` payload bytes (single transaction). */
bool i2c_write_regs(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint8_t n) {
    if (!wait_idle()) return false;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)(n + 1u) << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START | I2C_CR2_AUTOEND;
    if (!wait_flag(I2C_ISR_TXIS)) { bus_abort(); return false; }
    I2C1->TXDR = reg;
    for (uint8_t i = 0; i < n; i++) {
        if (!wait_flag(I2C_ISR_TXIS)) { bus_abort(); return false; }
        I2C1->TXDR = buf[i];
    }
    if (!wait_flag(I2C_ISR_STOPF)) { bus_abort(); return false; }
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

bool i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val) {
    return i2c_write_regs(addr7, reg, &val, 1);
}

bool i2c_read_regs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n) {
    /* Phase 1: write the register pointer (no STOP — manual RESTART follows). */
    if (!wait_idle()) return false;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | (1u << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START;
    if (!wait_flag(I2C_ISR_TXIS)) { bus_abort(); return false; }
    I2C1->TXDR = reg;
    if (!wait_flag(I2C_ISR_TC)) { bus_abort(); return false; }

    /* Phase 2: repeated start, read n bytes, auto STOP. */
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint8_t i = 0; i < n; i++) {
        if (!wait_flag(I2C_ISR_RXNE)) { bus_abort(); return false; }
        buf[i] = (uint8_t)I2C1->RXDR;
    }
    if (!wait_flag(I2C_ISR_STOPF)) { bus_abort(); return false; }
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}
