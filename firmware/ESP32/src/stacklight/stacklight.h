#pragma once

#include <stdint.h>

/* =============================================================================
 *  Stack light + buzzer driver.
 *
 *  Telemetry frames are tapped as they cross UART → WS. The relay extracts
 *  estop_sources and caution_sources bytes and feeds them into this module
 *  via stacklight_update_from_telemetry(); state is then derived per the
 *  table in architecture.md §Stack Light.
 *
 *  stacklight_tick(now_ms) must run regularly from loop() to drive the
 *  breathing/pulsing animations. ~50 Hz is plenty.
 * =============================================================================
 */

void stacklight_init(void);
void stacklight_update_from_telemetry(uint8_t estop_sources, uint8_t caution_sources,
                                      uint32_t now_ms);
void stacklight_tick(uint32_t now_ms);
