#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  Differential-drive odometry from wheel velocities.
 *
 *     v = (v_left + v_right) / 2
 *     ω = (v_right - v_left) / wheel_base
 *
 *  Pose integration:
 *     dx     = v · cos(theta) · dt
 *     dy     = v · sin(theta) · dt
 *     dtheta = ω · dt
 *
 *  encoders_tick() must run BEFORE odometry_tick() each control iteration so
 *  the wheel velocities are fresh.
 *
 *  Future-proofing: odometry_set_heading() lets a future EKF or BNO055 yaw
 *  reading override the integrated theta. The pose API keeps the same shape.
 * =============================================================================
 */

void  odometry_init(void);
void  odometry_tick(float dt_s);
void  odometry_reset(void);
void  odometry_set_heading(float theta_rad);

float odometry_x(void);
float odometry_y(void);
float odometry_theta(void);
float odometry_v(void);     /* m/s, signed */
float odometry_omega(void); /* rad/s, signed */

#endif /* ODOMETRY_H */
