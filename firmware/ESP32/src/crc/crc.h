#pragma once

#include <stdint.h>
#include <stddef.h>

/* CRC-16/CCITT-FALSE */
uint16_t crc16_compute(const uint8_t *data, size_t len);
