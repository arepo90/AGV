#pragma once

#include <stdint.h>

/* =============================================================================
 *  INA219 3S battery monitor on the local I2C bus (GPIO 6 SDA / 7 SCL).
 *
 *  The sensor moved here from the STM32 (whose I2C1 is disabled but kept in
 *  its tree). The ESP32 polls the bus voltage and main.cpp pushes each fresh
 *  reading to the STM32 as PKT_BATTERY — the firmware still owns the
 *  low-voltage caution/E-STOP decision and echoes the value in TLM_SENSORS.
 *
 *  battery_poll() rate-limits itself to BATT_POLL_HZ and re-probes an absent
 *  sensor each period, so a late-powered or re-plugged INA219 recovers
 *  without a reboot.
 * =============================================================================
 */

void battery_init();

/* True when *mv holds a fresh reading taken this call. */
bool battery_poll(uint32_t now_ms, uint16_t *mv);
