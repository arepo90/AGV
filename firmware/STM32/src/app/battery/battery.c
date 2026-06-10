#include "battery.h"
#include "config.h"
#include "log.h"
#include "mcu.h"
#include "types.h"

#if BATTERY_VIA_STM32_I2C
#include "i2c.h"

/* ---- Legacy direct-I2C driver (INA219 on I2C1 PB8/PB9) ---------------------
 * Disabled: the INA219 moved to the ESP32, which pushes readings over
 * PKT_BATTERY (path below). Kept compilable so the sensor can move back by
 * flipping BATTERY_VIA_STM32_I2C in config.h. */

/* ---- INA219 register subset --------------------------------------------- */
#define REG_CONFIG      0x00
#define REG_BUS_VOLTAGE 0x02
/* BRNG=1 (32 V), PGA/8, 12-bit bus+shunt ADC, bus-voltage continuous mode. */
#define CONFIG_VALUE    0x399Eu

static bool     s_present = false;
static uint16_t s_mv = 0;
static uint32_t s_last_ms = 0;

static bool write_config(void) {
    uint8_t b[2] = { (uint8_t)(CONFIG_VALUE >> 8), (uint8_t)CONFIG_VALUE };
    return i2c_write_regs(INA219_3S_I2C_ADDR, REG_CONFIG, b, 2);
}

/* Bus-voltage register: data in bits [15:3], LSB = INA219_BUS_LSB_UV. */
static bool read_mv(uint16_t *mv) {
    uint8_t b[2];
    if (!i2c_read_regs(INA219_3S_I2C_ADDR, REG_BUS_VOLTAGE, b, 2)) return false;
    uint16_t raw = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
    *mv = (uint16_t)(((uint32_t)(raw >> 3) * INA219_BUS_LSB_UV) / 1000u);
    return true;
}

void battery_init(void) {
#if DISABLE_BATTERY
    return;
#else
    i2c_init();
    s_mv = 0;
    s_present = write_config();   /* a clean config write proves the device acks */
    if (!s_present)
        log_record(LOG_MOD_BATTERY, LOG_SEV_WARN, LOG_CODE_BATTERY_I2C_FAIL, INA219_3S_I2C_ADDR);
    s_last_ms = mcu_now_ms();
#endif
}

void battery_tick(uint32_t now_ms) {
#if DISABLE_BATTERY
    (void)now_ms;
    return;
#else
    if (!s_present) return;
    if ((now_ms - s_last_ms) < (1000u / BATTERY_POLL_HZ)) return;
    s_last_ms = now_ms;

    uint16_t mv;
    if (!read_mv(&mv)) {
        log_record(LOG_MOD_BATTERY, LOG_SEV_WARN, LOG_CODE_BATTERY_I2C_FAIL, INA219_3S_I2C_ADDR);
        return;
    }
    /* Below the floor the rail is unpowered/unwired, not flat — report absent
     * so the safety monitor never trips on a disconnected sensor. */
    s_mv = (mv < BATTERY_MIN_VALID_MV) ? 0u : mv;
#endif
}

void battery_push_mv(uint16_t mv) { (void)mv; }   /* I2C owns the value here */

bool     battery_present(void) { return s_present && s_mv > 0u; }
uint16_t battery_mv(void)      { return s_mv; }

#else /* !BATTERY_VIA_STM32_I2C — ESP32-pushed source */

static uint16_t s_mv = 0;
static uint32_t s_last_push_ms = 0;

void battery_init(void) {
#if !DISABLE_BATTERY
    s_mv = 0;
    s_last_push_ms = mcu_now_ms();
#endif
}

void battery_push_mv(uint16_t mv) {
#if DISABLE_BATTERY
    (void)mv;
#else
    /* Below the floor the rail is unpowered/unwired, not flat — report absent
     * so the safety monitor never trips on a disconnected sensor. */
    s_mv = (mv < BATTERY_MIN_VALID_MV) ? 0u : mv;
    s_last_push_ms = mcu_now_ms();
#endif
}

void battery_tick(uint32_t now_ms) {
#if DISABLE_BATTERY
    (void)now_ms;
#else
    /* Staleness watch only — the ESP32 owns the polling. Going absent makes
     * the safety monitor release any active battery caution/E-STOP. */
    if (s_mv != 0u && (now_ms - s_last_push_ms) > BATTERY_PUSH_STALE_MS) {
        s_mv = 0u;
        log_record(LOG_MOD_BATTERY, LOG_SEV_WARN, LOG_CODE_BATTERY_STALE, 0);
    }
#endif
}

bool     battery_present(void) { return s_mv > 0u; }
uint16_t battery_mv(void)      { return s_mv; }

#endif /* BATTERY_VIA_STM32_I2C */
