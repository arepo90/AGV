#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  BNO055 9-DOF IMU driver over I2C1 (PB8 SCL / PB9 SDA, AF1).
 *
 *  Operates the BNO055 in NDOF fusion mode — onboard sensor fusion produces
 *  absolute orientation. We poll Euler angles and linear acceleration at
 *  IMU_READ_HZ (default 100 Hz).
 *
 *  Heading (yaw) is the primary output for use by the future complementary
 *  filter / EKF on top of encoder odometry. Pitch and roll are reported in
 *  telemetry for diagnostics; we don't act on them in nav code yet.
 *
 *  External 4.7 kΩ pull-ups required on SCL/SDA. I2C runs at 400 kHz fast
 *  mode (TIMINGR computed for 48 MHz APB1).
 *
 *  All I2C transactions are blocking with timeouts. Total time per cycle
 *  (read 12 bytes from 0x1A) ≈ 250 µs at 400 kHz — fine for 100 Hz polling.
 *  On bus-stuck conditions the driver attempts a recovery sequence and logs
 *  the event. Until first valid read, imu_has_data() returns false; the
 *  fusion code defers in that case.
 * =============================================================================
 */

void   imu_init(void);
void   imu_tick(uint32_t now_ms);

float  imu_yaw_deg(void);    /* heading 0..360°, BNO055 convention */
float  imu_pitch_deg(void);
float  imu_roll_deg(void);
float  imu_yaw_rad(void);    /* heading wrapped to [-π, +π], for math code */

/* Body-frame angular rate around Z (yaw-axis). Read from BNO055 raw gyro
 * (register 0x14). Positive sign matches the BNO055 NDOF yaw convention so
 * the EKF can subtract bias and integrate without flipping. Units: rad/s. */
float  imu_gyro_z_radps(void);

/* BNO055 calibration status: each field 0..3, with 3 = fully calibrated. */
uint8_t imu_calib_sys(void);
uint8_t imu_calib_gyro(void);
uint8_t imu_calib_accel(void);
uint8_t imu_calib_mag(void);

bool   imu_has_data(void);

#endif /* IMU_H */
