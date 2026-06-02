#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  BNO055 9-DOF IMU driver over hal/i2c (NDOF fusion mode).  (app tier)
 *
 *  Polls Euler angles + Z gyro + calibration status at IMU_READ_HZ. Heading
 *  (yaw) and gyro rate feed the complementary heading filter in odometry.c;
 *  pitch/roll are diagnostic only. If setup fails, imu_present() stays false
 *  and the fusion code falls back to encoder-only heading.
 *
 *  yaw_rad / gyro_z are returned in the encoder math convention (CCW-positive)
 *  via IMU_HEADING_SIGN, so consumers never juggle the BNO055 orientation.
 * =============================================================================
 */

void    imu_init(void);
void    imu_tick(uint32_t now_ms);

bool    imu_present(void);        /* setup succeeded */
bool    imu_has_data(void);       /* at least one good read */

float   imu_yaw_deg(void);
float   imu_pitch_deg(void);
float   imu_roll_deg(void);
float   imu_yaw_rad(void);        /* wrapped [-π, +π], CCW-positive */
float   imu_gyro_z_radps(void);   /* CCW-positive */

uint8_t imu_calib_sys(void);
uint8_t imu_calib_gyro(void);
uint8_t imu_calib_accel(void);
uint8_t imu_calib_mag(void);

#endif /* IMU_H */
