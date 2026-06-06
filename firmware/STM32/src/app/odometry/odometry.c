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

/* Heading Kalman filter state: x = [theta, gyro_bias], P symmetric 2x2.
 * s_omega_m is the gyro rate used in the latest predict (the filter input). */
static float s_bias = 0.0f;
static float s_omega_m = 0.0f;
static float s_P00 = 0.0f, s_P01 = 0.0f, s_P11 = 0.0f;
static bool  s_zupt = false;

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Predict: integrate the debiased gyro; P = F·P·Fᵀ + Q, F = [[1,-dt],[0,1]]. */
static void kf_predict(float omega_m, float dt) {
    s_omega_m = omega_m;
    s_theta = wrap_pi(s_theta + (omega_m - s_bias) * dt);
    float P00 = s_P00 - 2.0f * dt * s_P01 + dt * dt * s_P11 + HEADING_KF_Q_THETA * dt;
    float P01 = s_P01 - dt * s_P11;
    float P11 = s_P11 + HEADING_KF_Q_BIAS * dt;
    s_P00 = P00; s_P01 = P01; s_P11 = P11;
}

/* Correct against a yaw-rate measurement z (true rate model h = omega_m - bias).
 * H = [0,-1] → S = P11 + R. Encoder update and ZUPT share this path (ZUPT: z=0).
 * theta is nudged only through the cross-covariance P01. */
static void kf_correct(float z, float R) {
    float S = s_P11 + R;
    if (S <= 0.0f) return;
    float y = z - (s_omega_m - s_bias);          /* innovation */
    float k_theta = -s_P01 / S;
    float k_bias  = -s_P11 / S;
    s_theta = wrap_pi(s_theta + k_theta * y);
    s_bias += k_bias * y;
    float P00 = s_P00 - s_P01 * s_P01 / S;
    float P01 = s_P01 * (1.0f - s_P11 / S);
    float P11 = s_P11 * (1.0f - s_P11 / S);
    s_P00 = P00; s_P01 = P01; s_P11 = P11;
}

void odometry_init(void) {
    s_bias = 0.0f;
    s_P11  = HEADING_KF_P0_BIAS;
    odometry_reset();
}

/* Pose reset keeps the learned gyro bias + its covariance — the bias is a sensor
 * property, not part of the pose origin we are clearing. */
void odometry_reset(void) {
    s_x = s_y = s_theta = 0.0f;
    s_v = s_w = 0.0f;
    s_omega_m = 0.0f;
    s_zupt = false;
    s_P00 = HEADING_KF_P0_THETA;
    s_P01 = 0.0f;
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
    float enc_omega = (v_r - v_l) / (float)WHEEL_BASE_M;

    float theta_prev = s_theta;

    if (imu_present() && imu_has_data()) {
        float omega_m = imu_gyro_z_radps();
        kf_predict(omega_m, dt_s);

        /* Encoder yaw-rate correction, rejected when the gyro and encoders
         * disagree (wheel slip) so slip never corrupts the bias estimate. */
        if (fabsf((omega_m - s_bias) - enc_omega) < HEADING_SLIP_REJECT_RADPS)
            kf_correct(enc_omega, HEADING_KF_R_ENC);

        /* ZUPT: both wheels stopped and the chassis is still → true yaw rate is
         * zero, so the gyro reading is pure bias. Pins drift down each pause. */
        s_zupt = fabsf(v_l) < ZUPT_VEL_EPS_MPS &&
                 fabsf(v_r) < ZUPT_VEL_EPS_MPS &&
                 imu_is_still();
        if (s_zupt) kf_correct(0.0f, HEADING_KF_R_ZUPT);

        s_w = omega_m - s_bias;          /* debiased rate = best estimate of ω */
    } else {
        /* No IMU: pure encoder-differential heading (bias filter idle). */
        s_theta = wrap_pi(s_theta + enc_omega * dt_s);
        s_w = enc_omega;
        s_zupt = false;
    }

    /* Integrate position at the mid-tick heading (centred difference). */
    float theta_mid = wrap_pi(theta_prev + 0.5f * wrap_pi(s_theta - theta_prev));
    s_x += s_v * cosf(theta_mid) * dt_s;
    s_y += s_v * sinf(theta_mid) * dt_s;
#endif
}

float odometry_x(void)     { return s_x; }
float odometry_y(void)     { return s_y; }
float odometry_theta(void) { return s_theta; }
float odometry_v(void)     { return s_v; }
float odometry_omega(void) { return s_w; }

float odometry_gyro_bias_radps(void) { return s_bias; }
bool  odometry_bias_converged(void)  { return s_P11 < HEADING_KF_BIAS_CONVERGED; }
bool  odometry_zupt_active(void)     { return s_zupt; }
