#ifndef ODOMETRY_H
#define ODOMETRY_H

/* =============================================================================
 *  Pose estimation: complementary heading filter + dead reckoning.  (app tier)
 *
 *  Heading θ integrates the slip-immune BNO055 gyro (encoder-differential ω as
 *  fallback when the IMU is absent/uncalibrated), with a slow complementary
 *  pull toward the BNO055 absolute yaw to bound long-term drift. Position is
 *  dead-reckoned from encoder linear velocity and θ. One tunable
 *  (HEADING_COMP_ALPHA) — no covariance matrices.
 *
 *  Consumed by telemetry and the TRAJECTORY_FOLLOW pure-pursuit navigator.
 * =============================================================================
 */

void  odometry_init(void);
void  odometry_reset(void);
void  odometry_set_heading(float theta_rad);   /* external pose override */
void  odometry_tick(float dt_s);

float odometry_x(void);
float odometry_y(void);
float odometry_theta(void);
float odometry_v(void);       /* chassis linear velocity (m/s) */
float odometry_omega(void);   /* chassis angular velocity (rad/s) */

#endif /* ODOMETRY_H */
