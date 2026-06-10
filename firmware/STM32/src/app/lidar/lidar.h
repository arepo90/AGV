#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  LiDAR distance segments — Jetson-pushed, not a local sensor. (app tier)
 *
 *  The 2D LaserScan lives on the Jetson. Its lidar_node masks a hardcoded
 *  angular sector, bins the remainder into fixed angular intervals (average
 *  range per interval), and pushes the per-interval distances (mm) down to the
 *  STM32 in PKT_LIDAR_SEGMENTS. This module is purely the receive-side buffer —
 *  a distance-band safety input whose "sensor" is upstream over UART rather than
 *  on the I2C bus.
 *
 *  Freshness is fail-safe: if no segment frame has arrived within LIDAR_STALE_MS,
 *  every reading reports "clear" (LIDAR_VALID_MAX_MM) so a dead Jetson link can
 *  never latch an E-STOP. The threshold→caution/E-STOP policy lives in safety.c
 *  this module only stores and reports.
 * =============================================================================
 */

void     lidar_init(void);

/* Replace the stored segment set with the latest from the Jetson. n is clamped
 * to LIDAR_MAX_SEGMENTS; receipt time is stamped from mcu_now_ms(). Called from
 * the proto dispatch in main-loop context (not an ISR). */
void     lidar_set_segments(const uint16_t *mm, uint8_t n);

bool     lidar_is_fresh(void);          /* a segment frame arrived within LIDAR_STALE_MS */
uint8_t  lidar_segment_count(void);     /* fresh segment count; 0 when stale/none */
uint16_t lidar_segment_mm(uint32_t i);  /* segment i distance; LIDAR_VALID_MAX_MM if absent/stale */
uint16_t lidar_min_distance_mm(void);   /* min over fresh segments; clear sentinel if stale */

#endif /* LIDAR_H */
