#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Pose estimation: heading+bias Kalman filter + dead reckoning.  (app tier)
 *
 *  The MPU6050 gives no absolute heading (no magnetometer), so heading θ is
 *  fused by a 2-state linear Kalman filter [θ, gyro_bias]:
 *    - predict from the slip-immune gyro Z, debiased;
 *    - correct the bias from the encoder-differential yaw rate when not slipping;
 *    - correct the bias from a zero-velocity (ZUPT) pseudo-measurement whenever
 *      the robot is verified at rest, which bounds drift with no compass.
 *  With no IMU it degrades to pure encoder-differential heading. Position is
 *  dead-reckoned from encoder linear velocity and θ (no sensor observes x,y yet;
 *  promote to a full EKF when LiDAR/SLAM lands).
 *
 *  Consumed by telemetry and the TRAJECTORY_FOLLOW pure-pursuit navigator.
 * =============================================================================
 */

void  odometry_init(void);
void  odometry_reset(void);                    /* zero pose; keep the learned gyro bias */
void  odometry_set_heading(float theta_rad);   /* external pose override */
void  odometry_tick(float dt_s);

float odometry_x(void);
float odometry_y(void);
float odometry_theta(void);
float odometry_v(void);            /* chassis linear velocity (m/s) */
float odometry_omega(void);        /* chassis angular velocity (rad/s), debiased */

float odometry_gyro_bias_radps(void);   /* current KF gyro-bias estimate */
bool  odometry_bias_converged(void);     /* bias covariance has settled */
bool  odometry_zupt_active(void);        /* last tick applied a zero-velocity update */

#endif /* ODOMETRY_H */
