#include "i2cscan.h"
#include "config.h"
#include "mcu.h"
#include "uart.h"
#include "stm32f0xx.h"

/* TIMINGR for 100 kHz @ 48 MHz I2CCLK (ST CubeMX value). Deliberately the slow,
 * forgiving standard-mode timing — a long JST harness with weak pull-ups will
 * enumerate at 100 kHz even when it fails at 400 kHz. */
#define TIMINGR_100K_48MHZ   0x10805E89u
#define PROBE_TIMEOUT        100000u

/* ---- ASCII out over USART1 (idempotent; ESP32 forwards verbatim) ---------- */
static void tx(const char *s, uint16_t n) {
    while (!uart_send((const uint8_t *)s, n)) { /* slot ring full: let DMA drain */ }
    mcu_delay_ms(1);                            /* keep lines from racing the ring */
}
static void txs(const char *s) {
    uint16_t n = 0;
    while (s[n]) n++;
    tx(s, n);
}
static char *put_hex2(char *p, uint8_t v) {
    static const char H[] = "0123456789abcdef";
    *p++ = H[v >> 4];
    *p++ = H[v & 0x0F];
    return p;
}
static char *put_str(char *p, const char *s) {
    while (*s) *p++ = *s++;
    return p;
}

/* ---- raw bus level sampling (before AF: drive nothing, just read) ---------- */
static void pins_input(uint32_t pupdr8, uint32_t pupdr9) {
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);              /* input */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9))
                 | (pupdr8 << (8 * 2)) | (pupdr9 << (9 * 2));
    mcu_delay_ms(1);
}
#define SCL_HIGH() (!!(GPIOB->IDR & (1u << 8)))
#define SDA_HIGH() (!!(GPIOB->IDR & (1u << 9)))

/* Drive `low_pin` low (open-drain), set `read_pin` to input pull-up, return the
 * level read on `read_pin`. Two independent lines: pulling one low leaves the
 * other pulled high (returns 1). If the other drops too (returns 0), the two
 * lines are shorted together. */
static bool drive_low_read_other(uint8_t low_pin, uint8_t read_pin) {
    /* read_pin → input, pull-up. */
    GPIOB->MODER &= ~(3u << (read_pin * 2));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (read_pin * 2))) | (1u << (read_pin * 2));
    /* low_pin → open-drain output, driven low. */
    GPIOB->OTYPER |= (1u << low_pin);
    GPIOB->BSRR    = (1u << (low_pin + 16));            /* ODR bit = 0 */
    GPIOB->MODER   = (GPIOB->MODER & ~(3u << (low_pin * 2))) | (1u << (low_pin * 2));
    mcu_delay_ms(1);
    bool other_high = !!(GPIOB->IDR & (1u << read_pin));
    GPIOB->MODER &= ~(3u << (low_pin * 2));             /* release: back to input */
    return other_high;
}

/* Are PB8 (SCL) and PB9 (SDA) electrically independent? */
static void report_line_integrity(void) {
    bool sda_when_scl_low = drive_low_read_other(8, 9);   /* SCL low, read SDA */
    bool scl_when_sda_low = drive_low_read_other(9, 8);   /* SDA low, read SCL */
    if (!sda_when_scl_low || !scl_when_sda_low)
        txs("  -> SCL and SDA are SHORTED together (pulling one drags the other). "
            "Check for a solder bridge / crossed wire at PB8/PB9.\r\n");
}

/* ---- I2C bring-up: 48 MHz clock source + internal pull-ups + 100 kHz ------- */
static void bus_init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* Select SYSCLK (48 MHz) as the I2C1 kernel clock — reset default is HSI
     * (8 MHz), which would scale the TIMINGR bus speed by 1/6. (hal/i2c sets
     * this too; the scanner stays self-contained.) */
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

    /* PB8 SCL / PB9 SDA → AF1, open-drain, INTERNAL PULL-UP ENABLED. The weak
     * (~40 kΩ) internal pulls let the scan succeed even with no external 4.7 kΩ
     * on the harness — exactly what the Arduino's Wire library does for you. */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))
                 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9))
                 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFu << 0) | (0xFu << 4)))
                  | (1u << 0) | (1u << 4);

    I2C1->CR1     = 0;
    I2C1->TIMINGR = TIMINGR_100K_48MHZ;
    I2C1->CR1     = I2C_CR1_PE;
}

/* 0-byte write: address the device, AUTOEND issues STOP. A device is "present"
 * ONLY on a clean STOPF with no NACK/bus-error/arbitration-loss. Treating BERR
 * and ARLO as absent is what kills the phantom-address pattern a faulted bus
 * produces — otherwise a stray STOPF after an error reads as a ghost device. */
static bool probe(uint8_t addr7) {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | (0u << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
    for (uint32_t i = 0; i < PROBE_TIMEOUT; i++) {
        uint32_t isr = I2C1->ISR;
        if (isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) {
            I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
            return false;
        }
        if (isr & I2C_ISR_STOPF) { I2C1->ICR = I2C_ICR_STOPCF; return true; }
    }
    /* Hung mid-transfer (bus never released): force STOP so the next probe runs. */
    I2C1->CR2 |= I2C_CR2_STOP;
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    return false;
}

/* Probe one address and report the *raw* terminal ISR bits, so we see exactly
 * how the bus answers (clean ACK vs NACK vs bus-error vs hang) instead of a
 * yes/no. S=STOP(clean ack) N=NACK B=bus-error A=arb-lost -=timeout. */
static void probe_verbose(uint8_t addr7, const char *label) {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    I2C1->CR2 = ((uint32_t)(addr7 << 1) & I2C_CR2_SADD)
              | (0u << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
    uint32_t isr = 0;
    uint32_t stop = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_STOPF;
    uint32_t i = 0;
    for (; i < PROBE_TIMEOUT; i++) { isr = I2C1->ISR; if (isr & stop) break; }

    char line[48], *p = line;
    p = put_str(p, "  ");
    p = put_hex2(p, addr7);
    p = put_str(p, " (");
    p = put_str(p, label);
    p = put_str(p, "): ");
    if (i >= PROBE_TIMEOUT) p = put_str(p, "TIMEOUT");
    else {
        if (isr & I2C_ISR_NACKF) p = put_str(p, "NACK ");
        if (isr & I2C_ISR_BERR)  p = put_str(p, "BERR ");
        if (isr & I2C_ISR_ARLO)  p = put_str(p, "ARLO ");
        if ((isr & I2C_ISR_STOPF) && !(isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)))
            p = put_str(p, "ACK(clean)");
    }
    *p++ = '\r'; *p++ = '\n';
    tx(line, (uint16_t)(p - line));

    I2C1->CR2 |= I2C_CR2_STOP;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
}

/* Are external pull-ups on the harness? Sink each line with the internal
 * pull-DOWN: a real external 4.7 kΩ to 3V3 overpowers the ~40 kΩ pull-down and
 * the pin still reads high; nothing external and it reads low. */
static void report_bus_health(void) {
    pins_input(2u, 2u);                 /* pull-down */
    bool ext_scl = SCL_HIGH(), ext_sda = SDA_HIGH();
    pins_input(1u, 1u);                 /* pull-up: can the lines be released? */
    bool idle_scl = SCL_HIGH(), idle_sda = SDA_HIGH();

    char line[64], *p = line;
    const char *s = "bus: ext-pullup SCL=";
    while (*s) *p++ = *s++;
    *p++ = ext_scl ? 'Y' : 'N';
    s = " SDA="; while (*s) *p++ = *s++;
    *p++ = ext_sda ? 'Y' : 'N';
    s = " | idle SCL="; while (*s) *p++ = *s++;
    *p++ = idle_scl ? '1' : '0';
    s = " SDA="; while (*s) *p++ = *s++;
    *p++ = idle_sda ? '1' : '0';
    *p++ = '\r'; *p++ = '\n';
    tx(line, (uint16_t)(p - line));

    if (!ext_scl || !ext_sda)
        txs("  -> NO external pull-up detected (Arduino supplied its own). "
            "Add 4.7k to 3V3, or this scan's internal pulls will carry it.\r\n");
    if (!idle_scl || !idle_sda)
        txs("  -> a line is stuck LOW (short, or a device holding it). "
            "Bus cannot work until released.\r\n");
}

void i2cscan_run(void) {
    uart_init();   /* self-contained: scanner runs before proto_init() */

    for (;;) {
        txs("\r\n=== I2C SCAN v2  PB8 SCL / PB9 SDA  @100kHz (48MHz src) ===\r\n");

        report_bus_health();
        report_line_integrity();
        bus_init();

        /* Raw ISR decode of the addresses that should be on YOUR bus, plus an
         * odd/even control pair, so we see the real response per address. */
        txs("decode:\r\n");
        probe_verbose(INA219_3S_I2C_ADDR, "INA219 3S");
        probe_verbose(0x08, "even ctrl");
        probe_verbose(0x09, "odd ctrl");

        /* Direct bus scan. */
        char line[160], *p = line;
        const char *s = "found:";
        while (*s) *p++ = *s++;
        uint32_t n = 0;
        for (uint8_t a = 0x08; a <= 0x77; a++) {
            mcu_delay_ms(1);   /* let the bus fully settle between probes — a long/
                                * marginal harness gives phantom acks when hammered */
            if (probe(a)) { *p++ = ' '; p = put_hex2(p, a); n++; }
        }
        if (n == 0) { s = " (nothing)"; while (*s) *p++ = *s++; }
        *p++ = '\r'; *p++ = '\n';
        tx(line, (uint16_t)(p - line));

        txs("done\r\n");
        mcu_delay_ms(2000);
    }
}
