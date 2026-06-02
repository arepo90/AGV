#ifndef ANALOG_H
#define ANALOG_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  ADC1 multi-channel scan with DMA (DMA1 channel 1).  (hal tier)
 *
 *  Forward scan in channel-number order:
 *    IN2  PA2  M1 current   → idx 0      IN12 PC2  QTR5  → idx 6
 *    IN3  PA3  M2 current   → idx 1      IN13 PC3  QTR6  → idx 7
 *    IN4  PA4  QTR1         → idx 2      IN14 PC4  QTR7  → idx 8
 *    IN5  PA5  QTR2         → idx 3      IN15 PC5  QTR8  → idx 9
 *    IN10 PC0  QTR3         → idx 4
 *    IN11 PC1  QTR4         → idx 5
 *
 *  analog_tick() triggers a fresh scan on schedule and harvests the previous
 *  one's DMA buffer — non-blocking (a 10-channel scan is ~17 µs).
 * =============================================================================
 */

#define ANALOG_NUM_CHANNELS   10u
#define ANALOG_IDX_M1_CURRENT 0u
#define ANALOG_IDX_M2_CURRENT 1u
#define ANALOG_IDX_QTR_FIRST  2u
#define ANALOG_QTR_COUNT      8u

void     analog_init(void);
void     analog_tick(uint32_t now_ms);
bool     analog_has_data(void);             /* at least one scan completed */

uint16_t analog_qtr(uint32_t idx);          /* idx 0..7, 12-bit raw */
uint16_t analog_current_ma(uint32_t side);  /* side 0=M1(left), 1=M2(right) */

#endif /* ANALOG_H */
