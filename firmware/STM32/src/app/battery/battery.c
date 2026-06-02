#include "battery.h"
#include "config.h"
#include "i2c.h"
#include "log.h"
#include "mcu.h"
#include "types.h"

/* ---- INA219 register subset --------------------------------------------- */
#define REG_CONFIG      0x00
#define REG_BUS_VOLTAGE 0x02
/* BRNG=1 (32 V), PGA/8, 12-bit bus+shunt ADC, bus-voltage continuous mode. */
#define CONFIG_VALUE    0x399Eu

#define NUM_PACKS 2u

static const uint8_t ADDR[NUM_PACKS] = { INA219_3S_I2C_ADDR, INA219_6S_I2C_ADDR };

static bool     s_present[NUM_PACKS];
static uint16_t s_mv[NUM_PACKS];
static uint32_t s_last_ms = 0;

static bool write_config(uint8_t addr) {
    uint8_t b[2] = { (uint8_t)(CONFIG_VALUE >> 8), (uint8_t)CONFIG_VALUE };
    return i2c_write_regs(addr, REG_CONFIG, b, 2);
}

/* Bus-voltage register: data in bits [15:3], LSB = INA219_BUS_LSB_UV. */
static bool read_mv(uint8_t addr, uint16_t *mv) {
    uint8_t b[2];
    if (!i2c_read_regs(addr, REG_BUS_VOLTAGE, b, 2)) return false;
    uint16_t raw = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
    *mv = (uint16_t)(((uint32_t)(raw >> 3) * INA219_BUS_LSB_UV) / 1000u);
    return true;
}

void battery_init(void) {
#if DISABLE_BATTERY
    return;
#else
    i2c_init();
    for (uint32_t p = 0; p < NUM_PACKS; p++) {
        s_mv[p] = 0;
        s_present[p] = write_config(ADDR[p]);   /* a clean config write proves the device acks */
        if (!s_present[p])
            log_record(LOG_MOD_BATTERY, LOG_SEV_WARN, LOG_CODE_BATTERY_I2C_FAIL, ADDR[p]);
    }
    s_last_ms = mcu_now_ms();
#endif
}

void battery_tick(uint32_t now_ms) {
#if DISABLE_BATTERY
    (void)now_ms;
    return;
#else
    if ((now_ms - s_last_ms) < (1000u / BATTERY_POLL_HZ)) return;
    s_last_ms = now_ms;

    for (uint32_t p = 0; p < NUM_PACKS; p++) {
        if (!s_present[p]) continue;
        uint16_t mv;
        if (!read_mv(ADDR[p], &mv)) {
            log_record(LOG_MOD_BATTERY, LOG_SEV_WARN, LOG_CODE_BATTERY_I2C_FAIL, ADDR[p]);
            continue;
        }
        /* Below the floor the rail is unpowered/unwired, not flat — report absent
         * so the safety monitor never trips on a disconnected sensor. */
        s_mv[p] = (mv < BATTERY_MIN_VALID_MV) ? 0u : mv;
    }
#endif
}

bool     battery_present(uint32_t pack) { return (pack < NUM_PACKS) && s_present[pack] && s_mv[pack] > 0u; }
uint16_t battery_mv(uint32_t pack)      { return (pack < NUM_PACKS) ? s_mv[pack] : 0u; }
