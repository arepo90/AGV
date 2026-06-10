#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* =============================================================================
 *  Telemetry stream scheduler.  (app tier)
 *
 *  Replaces the old monolithic 128-byte frame with four rate-grouped streams,
 *  each its own packet type so the ESP32 stack-light tap can filter cheaply:
 *
 *    PKT_TLM_CORE     operational state + pose + chassis v/ω + currents + prox
 *                     (fast: TLM_CORE_HZ_MOVING navigating, TLM_CORE_HZ_IDLE idle)
 *    PKT_TLM_DRIVE    per-wheel targets/meas/duty/counts (TLM_DRIVE_HZ; PI tuning)
 *    PKT_TLM_SENSORS  load cells + battery + LiDAR tail (TLM_SENSORS_HZ)
 *    PKT_TLM_QTR      QTR raw + line position (control rate, LINE_FOLLOW/cal only)
 *
 *  telemetry_tick() sends whichever streams are due. Rates live in config.h.
 * =============================================================================
 */

void telemetry_init(void);
void telemetry_tick(uint32_t now_ms);

/* Indicator-light animation style (0 = pulse, 1 = snake), set via PARAM_LED_MODE
 * and reported in TLM_CORE so the ESP32 reads it from the telemetry tap. */
void telemetry_set_led_mode(uint8_t mode);

/* Distance-reactive ring config, packed (bit0 base off/white, bit1 fixed/responsive),
 * set via PARAM_LED_BASE / PARAM_LED_INDICATOR_MODE and echoed in TLM_CORE. */
void telemetry_set_indicator_cfg(uint8_t cfg);

#endif /* TELEMETRY_H */
