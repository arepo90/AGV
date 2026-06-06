#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  MPU6050 6-DOF IMU driver over hal/i2c (raw gyro + accel).  (app tier)
 *
 *  The MPU6050 has NO magnetometer, so there is no absolute heading reference —
 *  by design, since the IMU lives inside a closed metal enclosure next to two
 *  brushed DC motors where a magnetometer would be useless anyway. This driver
 *  therefore exposes only what the 6-DOF sensor can honestly measure:
 *
 *    - gyro Z rate (rad/s)  → the heading source for odometry's heading+bias
 *                             Kalman filter (slip-immune; its slowly-drifting
 *                             bias is estimated there, not here).
 *    - pitch / roll (deg)   → accel+gyro complementary tilt, gravity-referenced
 *                             and absolute; diagnostic only (flat-floor AGV).
 *    - is_still             → accel ≈ g and gyro small; used to gate the ZUPT
 *                             (zero-velocity) bias update in odometry.
 *
 *  No yaw is reported: yaw is owned by the fused estimate in odometry (theta).
 *  gyro_z is returned in the encoder math convention (CCW-positive) via
 *  IMU_HEADING_SIGN, so consumers never juggle the chip's axis orientation.
 *  If setup fails, imu_present() stays false and odometry falls back to
 *  encoder-only heading.
 * =============================================================================
 */

void    imu_init(void);
void    imu_tick(uint32_t now_ms);

bool    imu_present(void);        /* setup succeeded */
bool    imu_has_data(void);       /* at least one good read */
bool    imu_is_still(void);       /* accel ≈ 1 g and |gyro| small → likely at rest */

float   imu_gyro_z_radps(void);   /* latest raw Z rate, CCW-positive */
float   imu_pitch_deg(void);      /* complementary tilt, board frame */
float   imu_roll_deg(void);

#endif /* IMU_H */
