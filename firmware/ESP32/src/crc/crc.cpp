#include "crc.h"

static const uint16_t s_nibble_table[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
};

uint16_t crc16_compute(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        crc = (crc << 4) ^ s_nibble_table[((crc >> 12) ^ (b >> 4)) & 0x0Fu];
        crc = (crc << 4) ^ s_nibble_table[((crc >> 12) ^ (b     )) & 0x0Fu];
    }
    return crc;
}
