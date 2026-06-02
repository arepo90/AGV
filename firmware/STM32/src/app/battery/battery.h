#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Battery monitor — two INA219 modules reading bus voltage only.  (app tier)
 *
 *  Both sit directly on the I2C bus (0x40 = 3S, 0x41 = 6S), upstream of the TOF
 *  mux, so battery reads never contend with the round-robin ranging. No current
 *  sense: V+ and V- are tied to the rail; the shunt is unused. The 3S rail feeds
 *  the safety low-voltage monitor (caution → auto-clearing E-STOP); the 6S rail
 *  powers only the Jetson and is display/log-only. Percentage-of-charge is left
 *  to the consumers (Jetson panel node + GUI) from the raw millivolts.
 * =============================================================================
 */

#define BATTERY_3S 0u
#define BATTERY_6S 1u

void     battery_init(void);
void     battery_tick(uint32_t now_ms);

bool     battery_present(uint32_t pack);   /* BATTERY_3S / BATTERY_6S */
uint16_t battery_mv(uint32_t pack);        /* bus voltage in mV; 0 if absent */

#endif /* BATTERY_H */
