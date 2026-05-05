#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stddef.h>

/* =============================================================================
 * CRC-16/CCITT-FALSE in software.
 *
 *   poly  = 0x1021
 *   init  = 0xFFFF
 *   refin = false, refout = false, xorout = 0x0000
 *
 * The F051's CRC peripheral is fixed-polynomial CRC-32, so we can't use it for
 * CCITT. A 16-entry nibble table keeps this at ~20 cycles/byte — well under
 * 1% CPU at our packet rates.
 * =============================================================================
 */

void     crc_init(void);    /* no-op; kept for symmetry with future HW migration */
uint16_t crc16_compute(const uint8_t *data, size_t len);

#endif /* CRC_H */
