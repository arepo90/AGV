#include "odometry.h"
#include "config.h"
#include "ekf.h"
#include "encoders.h"
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
    ekf_init();
    odometry_reset();
}

void odometry_reset(void) {
    s_x = 0.0f; s_y = 0.0f; s_theta = 0.0f;
    s_v = 0.0f; s_w = 0.0f;
    ekf_reset();
    log_record(LOG_MOD_ODOMETRY, LOG_SEV_INFO, LOG_CODE_ODOMETRY_RESET, 0);
}

void odometry_set_heading(float theta_rad) {
    s_theta = wrap_pi(theta_rad);
    /* Note: this does NOT reseed the EKF — set_heading is currently used by
     * external pose overrides only. If a hard reset of fused heading is
     * needed, call odometry_reset(). */
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

    /* Step the EKF: it owns θ and gyro-bias estimates, fuses BNO055 yaw with
     * the IMU gyro propagation, and rejects outlier encoder ω as wheel slip. */
    float theta_pre = s_theta;
    ekf_tick(dt_s);
    s_theta = ekf_theta();
    /* Fused angular velocity (bias-corrected gyro when trusted, else encoder).
     * This is what the cascade outer loop reads — slip-resistant by design. */
    s_w = ekf_omega();

    /* Midpoint heading for position integration: average pre- and post-EKF θ
     * so the (x, y) trace uses the same accuracy improvement as a centred
     * difference. Unwrap diff via wrap_pi so wrap-around doesn't poison the
     * average. */
    float dtheta_step = wrap_pi(s_theta - theta_pre);
    float theta_mid   = wrap_pi(theta_pre + 0.5f * dtheta_step);

    s_x += s_v * cosf(theta_mid) * dt_s;
    s_y += s_v * sinf(theta_mid) * dt_s;
#endif
}

float odometry_x(void)     { return s_x; }
float odometry_y(void)     { return s_y; }
float odometry_theta(void) { return s_theta; }
float odometry_v(void)     { return s_v; }
float odometry_omega(void) { return s_w; }
