#ifndef HX711_H
#define HX711_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  HX711 24-bit load-cell amplifier — 4 cells in parallel.
 *
 *  Wiring (all on GPIOB):
 *    PB10  shared SCK
 *    PB12  DOUT — front-left  (corner 0)
 *    PB13  DOUT — front-right (corner 1)
 *    PB14  DOUT — rear-left   (corner 2)
 *    PB15  DOUT — rear-right  (corner 3)
 *
 *  Per the HX711 datasheet: a single SCK clocks all four chips simultaneously,
 *  so 25 SCK pulses harvest one 24-bit reading from each corner in parallel.
 *  Total bit-bang time at modest SCK rate (~50 kHz) ≈ 500 µs per read.
 *
 *  The 25th pulse selects channel A gain 128 for the next conversion.
 *
 *  Read scheduling lives in this module; main loop just calls hx711_tick().
 *  Rate switches automatically with the AGV state (30 Hz when stationary,
 *  2 Hz when moving — see config.h).
 *
 *  Tare and per-cell calibration are runtime-tunable (defaults in config.h).
 * =============================================================================
 */

#define HX711_NUM_CORNERS    4u

void     hx711_init(void);
void     hx711_tick(uint32_t now_ms);

int32_t  hx711_raw(uint32_t corner);          /* signed 24-bit, sign-extended */
float    hx711_kg(uint32_t corner);           /* (raw - offset) × scale */
float    hx711_total_kg(void);

void     hx711_set_offset(uint32_t corner, int32_t offset_counts);
void     hx711_set_scale(uint32_t corner, float kg_per_count);
int32_t  hx711_offset(uint32_t corner);
float    hx711_scale(uint32_t corner);

bool     hx711_has_data(void);

/* Begin a tare cycle — averages N readings into the offset for each corner,
 * then resumes normal operation. Triggered by CMD_START_TARE. */
void     hx711_start_tare(void);
bool     hx711_tare_in_progress(void);

#endif /* HX711_H */
