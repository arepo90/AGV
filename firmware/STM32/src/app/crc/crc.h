#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stddef.h>

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Used by the frame layer
 * (proto.c). */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#endif /* CRC_H */
