#ifndef LOADCELLS_H
#define LOADCELLS_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  HX711 load-cell amplifiers (4 corners, shared SCK).  (app tier)
 *
 *  PB10 = SCK (output), PB12..PB15 = DOUT for corners 0..3 (inputs). All four
 *  share the clock and are sampled in parallel per clock pulse — deterministic
 *  read time. Bit-bang with IRQs masked so SCK never stays high > 60 µs.
 *
 *  loadcells_tick(now_ms, moving): read rate depends on whether the AGV is
 *  navigating (slow) or idle/weight-setting (fast). Offsets/scales are RAM-only
 *  (workstation persists and re-sends). This is the sole user of its GPIO, so
 *  it touches the registers directly rather than via a HAL shim.
 * =============================================================================
 */

void    loadcells_init(void);
void    loadcells_tick(uint32_t now_ms, bool moving);
bool    loadcells_has_data(void);

float   loadcells_kg(uint32_t corner);     /* corner 0..3 */
float   loadcells_total_kg(void);
void    loadcells_set_offset(uint32_t corner, int32_t counts);
void    loadcells_set_scale(uint32_t corner, float scale);

void    loadcells_start_tare(void);
bool    loadcells_tare_in_progress(void);

#endif /* LOADCELLS_H */
