#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <stdint.h>

/* Pose estimation: encoder dead-reckoning. (app tier)
 * Heading θ and position (x, y) are integrated from wheel encoder velocities
 * only — no IMU. Position is dead-reckoned; no sensor observes x, y, or θ. */

void  odometry_init(void);
void  odometry_reset(void);
void  odometry_set_heading(float theta_rad);
void  odometry_tick(float dt_s);

float odometry_x(void);
float odometry_y(void);
float odometry_theta(void);
float odometry_v(void);
float odometry_omega(void);

#endif /* ODOMETRY_H */
