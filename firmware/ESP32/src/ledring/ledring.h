#pragma once

#include <stdint.h>

/* =============================================================================
 *  Indicator LED rings (WS2812B) — the AGV's stack-light replacement.
 *
 *  TLM_CORE frames are tapped as they cross UART → USB. The relay feeds
 *  estop_sources / caution_sources (→ state → colour) and led_mode (→ pulse or
 *  snake animation) into this module. All four rings render the same state; the
 *  animation uses a normalised phase so rings of different lengths stay in sync.
 *
 *  ledring_tick(now_ms) must run regularly from loop(); it rate-limits its own
 *  rendering (LED_RING_REFRESH_HZ) since show() briefly blocks.
 *
 *  Colours: NORMAL green, CAUTION yellow, ESTOP/INIT/DISCONNECT red.
 * =============================================================================
 */

void ledring_init(void);
void ledring_update_from_telemetry(uint16_t estop_sources, uint16_t caution_sources,
                                   uint8_t led_mode, uint32_t now_ms);
void ledring_tick(uint32_t now_ms);
