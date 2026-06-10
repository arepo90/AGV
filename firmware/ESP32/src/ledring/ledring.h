#pragma once

#include <stdint.h>

/* =============================================================================
 *  Indicator LED rings (WS2812B) — the AGV's stack-light replacement.
 *
 *  Telemetry frames are tapped as they cross UART → USB:
 *    - TLM_CORE feeds estop/caution (→ state → colour), led_mode (pulse/snake),
 *      the IR proximity bits, and the indicator-ring config byte.
 *    - TLM_SENSORS feeds the LiDAR segment tail.
 *
 *  Three rings (the two small + one big) render the shared system state with the
 *  pulse/snake animation. The remaining big ring (LED_INDICATOR_RING) is instead
 *  a top-down obstacle display: a base layer (off/white) plus per-sensor indicator
 *  points whose colour + span follow the measured distance, overlaps resolved to
 *  the nearest reading. See config.h for the wiring-dependent layout.
 *
 *  ledring_tick(now_ms) must run regularly from loop(); it rate-limits its own
 *  rendering (LED_RING_REFRESH_HZ) since show() briefly blocks. The reactive ring
 *  eases toward new distances each frame so 5 Hz sensor telemetry looks fluid.
 * =============================================================================
 */

void ledring_init(void);

/* From the TLM_CORE tap: state colour + animation + indicator-ring config + IR bits. */
void ledring_update_from_telemetry(uint16_t estop_sources, uint16_t caution_sources,
                                   uint8_t led_mode, uint8_t indicator_cfg,
                                   uint16_t proximity_bits, uint32_t now_ms);

/* From the TLM_SENSORS tap: LiDAR segment tail (mm per angular interval;
 * n = 0 when no fresh LiDAR data). */
void ledring_update_sensors(const uint16_t *lidar_mm, uint8_t lidar_n);

void ledring_tick(uint32_t now_ms);
