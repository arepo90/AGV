#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  I2C1 master (PB8 SCL / PB9 SDA, AF1), 400 kHz fast mode.  (hal tier)
 *
 *  Blocking transfers with a cycle-bounded timeout. Returns false on NACK,
 *  arbitration loss, bus error, or timeout — the caller decides what to log.
 *  External 4.7 kΩ pull-ups are assumed (open-drain, no internal pulls).
 * =============================================================================
 */

void i2c_init(void);
bool i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val);
bool i2c_write_regs(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint8_t n);
bool i2c_write(uint8_t addr7, const uint8_t *buf, uint8_t n);   /* raw, no register byte */
bool i2c_read_regs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n);

#endif /* I2C_H */
