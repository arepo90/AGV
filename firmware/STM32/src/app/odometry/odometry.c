#include "odometry.h"
#include "config.h"
#include "encoders.h"
#include "imu.h"
#include "log.h"
#include "types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float s_x = 0.0f, s_y = 0.0f, s_theta = 0.0f;
static float s_v = 0.0f, s_w = 0.0f;

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void odometry_init(void) {
    odometry_reset();
}

void odometry_reset(void) {
    s_x = s_y = s_theta = 0.0f;
    s_v = s_w = 0.0f;
    log_record(LOG_MOD_ODOMETRY, LOG_SEV_INFO, LOG_CODE_ODOMETRY_RESET, 0);
}

void odometry_set_heading(float theta_rad) {
    s_theta = wrap_pi(theta_rad);
}

void odometry_tick(float dt_s) {
#if DISABLE_ODOMETRY
    (void)dt_s;
    return;
#else
    if (dt_s <= 0.0f) return;

    float v_l = encoders_velocity_mps(SIDE_LEFT);
    float v_r = encoders_velocity_mps(SIDE_RIGHT);
    s_v = 0.5f * (v_l + v_r);

    /* Heading rate: gyro is slip-immune; fall back to wheel differential when
     * the IMU is absent or its gyro hasn't calibrated yet. */
    bool use_gyro = imu_present() && imu_has_data() &&
                    imu_calib_gyro() >= HEADING_MIN_GYRO_CALIB;
    float rate = use_gyro ? imu_gyro_z_radps()
                          : (v_r - v_l) / (float)WHEEL_BASE_M;

    float theta_prev = s_theta;
    float theta_prop = theta_prev + rate * dt_s;     /* gyro/encoder propagation */

    /* Complementary correction: slow pull toward the BNO055 absolute yaw to
     * bound drift, gated on the fusion (system) calibration being trustworthy. */
    if (imu_present() && imu_has_data() && imu_calib_sys() >= HEADING_MIN_YAW_CALIB) {
        float err = wrap_pi(imu_yaw_rad() - theta_prop);
        s_theta = wrap_pi(theta_prop + (1.0f - HEADING_COMP_ALPHA) * err);
    } else {
        s_theta = wrap_pi(theta_prop);
    }
    s_w = rate;

    /* Integrate position at the mid-tick heading (centred difference). */
    float theta_mid = wrap_pi(theta_prev + 0.5f * rate * dt_s);
    s_x += s_v * cosf(theta_mid) * dt_s;
    s_y += s_v * sinf(theta_mid) * dt_s;
#endif
}

float odometry_x(void)     { return s_x; }
float odometry_y(void)     { return s_y; }
float odometry_theta(void) { return s_theta; }
float odometry_v(void)     { return s_v; }
float odometry_omega(void) { return s_w; }
