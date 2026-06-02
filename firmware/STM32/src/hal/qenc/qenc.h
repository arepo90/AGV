#ifndef QENC_H
#define QENC_H

#include <stdint.h>

/* =============================================================================
 *  Quadrature encoder timers.  (hal tier)
 *
 *  Channel 0 = LEFT  : TIM2 on PA0/PA1 (AF2)
 *  Channel 1 = RIGHT : TIM3 on PA6/PA7 (AF1)
 *
 *  Both run in hardware encoder mode (×4 decoding) with ARR = 0xFFFF so each
 *  reads as a free-running 16-bit up/down counter. app/encoders.c takes signed
 *  16-bit differences (wrap-safe) and converts to velocity/distance.
 * =============================================================================
 */

void     qenc_init(void);
uint16_t qenc_raw(uint8_t ch);   /* low 16 bits of the counter; ch 0=left, 1=right */

#endif /* QENC_H */
