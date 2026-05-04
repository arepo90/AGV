#pragma once

#include <stdint.h>
#include <stddef.h>

/* CRC-16/CCITT-FALSE — same implementation as STM32 firmware/STM32/src/crc.c.
 * Both endpoints MUST use the identical algorithm or every frame fails. */

uint16_t crc16_compute(const uint8_t *data, size_t len);
