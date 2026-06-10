#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Battery monitor — one INA219 reading 3S bus voltage only.  (app tier)
 *
 *  The INA219 hangs off the ESP32 (its I2C, GPIO6/7); the ESP32 polls it and
 *  pushes each reading here as PKT_BATTERY (u16 mV), routed in by proto. The
 *  3S rail feeds the safety low-voltage monitor (caution → auto-clearing
 *  E-STOP); a stale push stream reports the battery absent (fail-safe, like
 *  the LiDAR monitor). Percentage-of-charge is left to the consumers (Jetson
 *  panel node + GUI) from the raw millivolts.
 *
 *  The legacy direct-I2C driver (INA219 on I2C1 PB8/PB9, no current sense)
 *  is kept behind BATTERY_VIA_STM32_I2C in config.h.
 * =============================================================================
 */

void     battery_init(void);
void     battery_tick(uint32_t now_ms);
void     battery_push_mv(uint16_t mv); /* PKT_BATTERY ingest (ignored in I2C mode) */

bool     battery_present(void);
uint16_t battery_mv(void);          /* 3S bus voltage in mV; 0 if absent */

#endif /* BATTERY_H */
