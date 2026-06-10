#ifndef I2CSCAN_H
#define I2CSCAN_H

/* =============================================================================
 *  I2C bus bench diagnostic (temporary tool — not part of the running system).
 *
 *  Independent of hal/i2c on purpose: it brings the bus up itself so it can
 *  enable the MCU's internal pull-ups and pick the I2C clock source explicitly,
 *  isolating the two things that differ between the STM32 and the Arduino test.
 *
 *  It runs three checks and prints them as plain ASCII over USART1 (the ESP32
 *  forwards bytes verbatim, so it appears on /dev/agv-esp32 at 921600 — open it
 *  with `screen /dev/agv-esp32 921600` or `picocom -b 921600 /dev/agv-esp32`):
 *
 *    1. bus health — are external pull-ups present? are the lines released?
 *    2. full address scan (0x08..0x77)
 *    3. behind-mux scan (select each TCA9548A channel, probe 0x29)
 *
 *  Enable with I2C_SCAN=1 in config.h; main() then calls this right after the
 *  clock/UART come up and never returns. Loops every ~2 s so you can attach the
 *  terminal at any time.
 * =============================================================================
 */
void i2cscan_run(void);

#endif /* I2CSCAN_H */
