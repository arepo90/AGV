#include "ekf.h"
#include "config.h"
#include "encoders.h"
#include "imu.h"
#include "log.h"
#include "types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* State: x = [θ, b_g]. Covariance P is 2×2 symmetric, stored as 3 scalars. */
static float s_theta = 0.0f;
static float s_bias  = 0.0f;
static float s_P00, s_P01, s_P11;
static bool     s_slip_last  = false;
static uint32_t s_slip_count = 0;

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static float encoder_omega(void) {
    float v_l = encoders_velocity_mps(ENC_LEFT);
    float v_r = encoders_velocity_mps(ENC_RIGHT);
    return (v_r - v_l) / WHEEL_BASE_M;
}

void ekf_init(void) {
    ekf_reset();
}

void ekf_reset(void) {
    s_theta = 0.0f;
    s_bias  = EKF_INITIAL_GYRO_BIAS;
    s_P00 = EKF_P0_THETA;
    s_P01 = 0.0f;
    s_P11 = EKF_P0_BIAS;
    s_slip_last  = false;
    s_slip_count = 0;
}

void ekf_tick(float dt_s) {
    if (dt_s <= 0.0f) return;

#if DISABLE_HEADING_FUSION
    /* Fusion disabled — fall back to pure encoder propagation so callers
     * still get something sensible. Covariance is untouched (not meaningful
     * in this mode). */
    s_theta = wrap_pi(s_theta + encoder_omega() * dt_s);
    s_slip_last = false;
    return;
#else
    bool imu_ok       = imu_has_data();
    bool gyro_trusted = imu_ok && (imu_calib_gyro() >= EKF_MIN_GYRO_CALIB);
    bool yaw_trusted  = imu_ok && (imu_calib_sys()  >= EKF_MIN_YAW_CALIB);

    float w_imu = imu_gyro_z_radps();
    float w_enc = encoder_omega();

    /* ---- 1. Predict --------------------------------------------------------
     * Use gyro − bias if we trust the gyro; otherwise fall back to encoder ω
     * so the integrator at least matches dead-reckoning. */
    float w_pred = gyro_trusted ? (w_imu - s_bias) : w_enc;
    s_theta = wrap_pi(s_theta + w_pred * dt_s);

    /* F = [[1, -dt], [0, 1]];  P ← F·P·Fᵀ + Q
     *   P00' = P00 - 2·dt·P01 + dt²·P11 + Q_θ·dt
     *   P01' = P01 -    dt·P11
     *   P11' = P11                       + Q_b·dt
     */
    {
        float dt   = dt_s;
        float P00n = s_P00 - 2.0f * dt * s_P01 + dt * dt * s_P11 + EKF_Q_THETA * dt;
        float P01n = s_P01 - dt * s_P11;
        float P11n = s_P11 + EKF_Q_BIAS * dt;
        s_P00 = P00n;
        s_P01 = P01n;
        s_P11 = P11n;
    }

    s_slip_last = false;

    /* ---- 2. Update with BNO055 absolute yaw --------------------------------
     * z = θ + v_yaw,  H = [1, 0],  R = R_yaw,  S = P00 + R
     * K = [P00/S, P01/S];  innovation y = wrap(z - θ)
     */
    if (yaw_trusted) {
        float z_yaw = imu_yaw_rad();
        float y     = wrap_pi(z_yaw - s_theta);
        float S     = s_P00 + EKF_R_YAW;
        float K0    = s_P00 / S;
        float K1    = s_P01 / S;
        s_theta = wrap_pi(s_theta + K0 * y);
        s_bias  = s_bias  + K1 * y;
        float P00_old = s_P00;
        float P01_old = s_P01;
        s_P00 = P00_old - K0 * P00_old;
        s_P01 = P01_old - K0 * P01_old;
        s_P11 = s_P11   - K1 * P01_old;
    }

    /* ---- 3. Update with encoder ω ------------------------------------------
     * h(x) = ω_imu - b_g,  H = [0, -1],  R = R_enc,  S = P11 + R
     * K = [-P01/S, -P11/S]; innovation y = z_enc - (ω_imu - b_g)
     *
     * Chi-squared outlier gate: y² > N²·S → reject (wheel slip suspected).
     * Only run the update when the gyro prediction is meaningful (otherwise
     * we'd be learning bias against an unobserved gyro). */
    if (gyro_trusted) {
        float h = w_imu - s_bias;
        float y = w_enc - h;
        float S = s_P11 + EKF_R_OMEGA_ENC;

        if (y * y > (EKF_SLIP_GATE_N * EKF_SLIP_GATE_N) * S) {
            s_slip_last = true;
            s_slip_count++;
            log_record(LOG_MOD_ODOMETRY, LOG_SEV_WARN, LOG_CODE_EKF_WHEEL_SLIP, 0);
        } else {
            float K0 = -s_P01 / S;
            float K1 = -s_P11 / S;
            s_theta = wrap_pi(s_theta + K0 * y);
            s_bias  = s_bias  + K1 * y;
            float P01_old = s_P01;
            float P11_old = s_P11;
            s_P00 = s_P00 - (P01_old * P01_old) / S;
            s_P01 = P01_old * (1.0f - P11_old / S);
            s_P11 = P11_old * (1.0f - P11_old / S);
        }
    }
#endif
}

float    ekf_theta(void)         { return s_theta; }
float    ekf_gyro_bias(void)     { return s_bias; }
bool     ekf_slip_detected(void) { return s_slip_last; }
uint32_t ekf_slip_count(void)    { return s_slip_count; }

float ekf_omega(void) {
    /* Prefer the bias-corrected gyro when it's trustworthy — the gyro reads
     * the body's actual rotation regardless of wheel contact, so it's the
     * better estimate when a wheel is slipping. Otherwise fall back to the
     * encoder, which is at least well-defined under nominal conditions. */
    if (imu_has_data() && imu_calib_gyro() >= EKF_MIN_GYRO_CALIB) {
        return imu_gyro_z_radps() - s_bias;
    }
    return encoder_omega();
}
