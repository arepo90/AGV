#include "battery.h"
#include "config.h"
#include <Wire.h>

/* ---- INA219 register subset (mirrors the STM32's legacy driver) ---------- */
static const uint8_t  REG_CONFIG      = 0x00;
static const uint8_t  REG_BUS_VOLTAGE = 0x02;
/* BRNG=1 (32 V), PGA/8, 12-bit bus+shunt ADC, bus-voltage continuous mode. */
static const uint16_t CONFIG_VALUE    = 0x399E;

static bool     s_present = false;
static uint32_t s_last_ms = 0;

static bool write_config() {
    Wire.beginTransmission(BATT_INA219_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write((uint8_t)(CONFIG_VALUE >> 8));
    Wire.write((uint8_t)CONFIG_VALUE);
    return Wire.endTransmission() == 0;
}

/* Bus-voltage register: data in bits [15:3], LSB = 4 mV. */
static bool read_mv(uint16_t *mv) {
    Wire.beginTransmission(BATT_INA219_ADDR);
    Wire.write(REG_BUS_VOLTAGE);
    if (Wire.endTransmission(false) != 0) return false;   /* repeated start */
    if (Wire.requestFrom((uint8_t)BATT_INA219_ADDR, (uint8_t)2) != 2) return false;
    uint16_t raw = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
    *mv = (uint16_t)((raw >> 3) * 4u);
    return true;
}

void battery_init() {
    Wire.begin(BATT_I2C_SDA_PIN, BATT_I2C_SCL_PIN);
    s_present = write_config();   /* a clean config write proves the device acks */
}

bool battery_poll(uint32_t now_ms, uint16_t *mv) {
    if ((uint32_t)(now_ms - s_last_ms) < (1000u / BATT_POLL_HZ)) return false;
    s_last_ms = now_ms;

    if (!s_present) {
        s_present = write_config();
        if (!s_present) return false;
    }
    if (!read_mv(mv)) {
        s_present = false;        /* re-probe next period */
        return false;
    }
    return true;
}
