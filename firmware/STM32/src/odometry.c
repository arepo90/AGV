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
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

void odometry_init(void) {
    odometry_reset();
}

void odometry_reset(void) {
    s_x = 0.0f; s_y = 0.0f; s_theta = 0.0f;
    s_v = 0.0f; s_w = 0.0f;
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

    float v_l = encoders_velocity_mps(ENC_LEFT);
    float v_r = encoders_velocity_mps(ENC_RIGHT);

    s_v = 0.5f * (v_l + v_r);
    s_w = (v_r - v_l) / WHEEL_BASE_M;

    /* Midpoint integration on theta for slightly better accuracy on curves. */
    float dtheta = s_w * dt_s;
    float theta_mid = s_theta + 0.5f * dtheta;

    s_x += s_v * cosf(theta_mid) * dt_s;
    s_y += s_v * sinf(theta_mid) * dt_s;

    float theta_pred = s_theta + dtheta;

#if !DISABLE_HEADING_FUSION
    /* Complementary filter: trust encoder dead-reckoning for high-frequency
     * (smooth, no quantisation), pull toward BNO055 absolute yaw for low-
     * frequency (corrects drift). Gated on BNO055 system calibration so we
     * don't pollute the heading with garbage during initial mag-cal. */
    if (imu_has_data() && imu_calib_sys() >= HEADING_FUSION_MIN_CALIB) {
        float imu_h = imu_yaw_rad();
        float diff = imu_h - theta_pred;
        /* Wrap diff into [-π, π] so we always correct the short way around. */
        while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
        while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
        theta_pred += (1.0f - HEADING_FUSION_ALPHA) * diff;
    }
#endif

    s_theta = wrap_pi(theta_pred);
#endif
}

float odometry_x(void)     { return s_x; }
float odometry_y(void)     { return s_y; }
float odometry_theta(void) { return s_theta; }
float odometry_v(void)     { return s_v; }
float odometry_omega(void) { return s_w; }
