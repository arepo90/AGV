#include "lidar.h"
#include "config.h"
#include "mcu.h"

/* Stored segment set from the most recent PKT_LIDAR_SEGMENTS frame. */
static uint16_t s_mm[LIDAR_MAX_SEGMENTS];
static uint8_t  s_count   = 0;
static uint32_t s_last_rx = 0;
static bool     s_ever_rx = false;

void lidar_init(void) {
    s_count   = 0;
    s_last_rx = 0;
    s_ever_rx = false;
}

void lidar_set_segments(const uint16_t *mm, uint8_t n) {
    if (n > LIDAR_MAX_SEGMENTS) n = LIDAR_MAX_SEGMENTS;
    for (uint8_t i = 0; i < n; i++) s_mm[i] = mm[i];
    s_count   = n;
    s_last_rx = mcu_now_ms();
    s_ever_rx = true;
}

bool lidar_is_fresh(void) {
    if (!s_ever_rx || s_count == 0) return false;
    return (uint32_t)(mcu_now_ms() - s_last_rx) <= LIDAR_STALE_MS;
}

uint8_t lidar_segment_count(void) {
    return lidar_is_fresh() ? s_count : 0u;
}

uint16_t lidar_segment_mm(uint32_t i) {
    if (!lidar_is_fresh() || i >= s_count) return (uint16_t)LIDAR_VALID_MAX_MM;
    return s_mm[i];
}

uint16_t lidar_min_distance_mm(void) {
    if (!lidar_is_fresh()) return (uint16_t)LIDAR_VALID_MAX_MM;
    uint16_t best = (uint16_t)LIDAR_VALID_MAX_MM;
    for (uint8_t i = 0; i < s_count; i++)
        if (s_mm[i] < best) best = s_mm[i];
    return best;
}
