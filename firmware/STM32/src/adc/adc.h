#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  ADC1 multi-channel scan with DMA (DMA1 channel 1).
 *
 *  Channel sequence (forward scan, in channel-number order):
 *     ADC_IN2  PA2  M1 current sense     → s_buf[0]
 *     ADC_IN3  PA3  M2 current sense     → s_buf[1]
 *     ADC_IN4  PA4  QTR sensor 1         → s_buf[2]
 *     ADC_IN5  PA5  QTR sensor 2         → s_buf[3]
 *     ADC_IN10 PC0  QTR sensor 3         → s_buf[4]
 *     ADC_IN11 PC1  QTR sensor 4         → s_buf[5]
 *     ADC_IN12 PC2  QTR sensor 5         → s_buf[6]
 *     ADC_IN13 PC3  QTR sensor 6         → s_buf[7]
 *     ADC_IN14 PC4  QTR sensor 7         → s_buf[8]
 *     ADC_IN15 PC5  QTR sensor 8         → s_buf[9]
 *
 *  Operation: each adc_tick() that lands on a scheduled period triggers a
 *  fresh scan and harvests the previous one's DMA buffer. This double-pass
 *  keeps the main loop non-blocking — a scan completes in ~17 µs but we don't
 *  even spin on it, just check DMA TC flag next tick.
 *
 *  Sample time: 7.5 ADC clock cycles (low-impedance signals — current sense
 *  is buffered internally by the Pololu G2; QTR-8A outputs are buffered too).
 * =============================================================================
 */

#define ADC_NUM_CHANNELS    10u
#define ADC_IDX_M1_CURRENT  0u
#define ADC_IDX_M2_CURRENT  1u
#define ADC_IDX_QTR_FIRST   2u   /* QTR 0..7 at indices 2..9 */
#define ADC_IDX_QTR_COUNT   8u

void     adc_init(void);
void     adc_tick(uint32_t now_ms);

/* Latest converted values. Stale by at most one scan period. */
uint16_t adc_raw(uint32_t idx);                  /* 12-bit raw count */
uint16_t adc_motor_current_ma(uint32_t side);    /* side: 0=M1, 1=M2 */
uint16_t adc_qtr(uint32_t idx);                  /* idx: 0..7 */

/* Has at least one scan completed since boot? Used to defer monitors that
 * shouldn't fire on uninitialised data. */
bool     adc_has_data(void);

#endif /* ADC_H */
