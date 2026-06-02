#ifndef EKF_H
#define EKF_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  2-state EKF for heading and gyro-bias estimation.
 *
 *    x  = [θ,  b_g]ᵀ
 *    f(x, u, dt):    θ_{k+1} = θ_k + (ω_imu_k - b_g_k) · dt
 *                    b_{k+1} = b_k          (driven by Q only — random walk)
 *
 *    h_yaw(x) = θ                   z_yaw = BNO055 absolute yaw
 *    h_enc(x) = ω_imu - b_g         z_enc = encoder ω (cross-checks bias;
 *                                          chi²-gated to reject wheel slip)
 *
 *  The IMU gyro is the **input** to the propagation, not a measurement —
 *  this is the standard strapdown-INS formulation. Encoder ω is then a
 *  measurement of (ω_imu - b_g), which is what teaches the EKF the gyro
 *  bias. When a wheel slips, the encoder measurement becomes an outlier
 *  relative to the IMU prediction, the chi-squared gate rejects it, and
 *  the EKF heading is unaffected by the slip transient.
 *
 *  Position (x, y) is NOT in the state — no sensor observes it, so it would
 *  reduce to pure dead-reckoning anyway. odometry.c integrates (x, y) using
 *  v from encoders and θ from this module.
 *
 *  Gating: predict always runs (cheap, keeps θ moving). Yaw update needs
 *  imu_calib_sys() ≥ EKF_MIN_YAW_CALIB; encoder update needs
 *  imu_calib_gyro() ≥ EKF_MIN_GYRO_CALIB (so the gyro prediction is sane).
 *  Without IMU at all, the EKF falls back to encoder-driven propagation so
 *  callers always get a sensible θ.
 *
 *  ekf_init() / ekf_reset() seed from EKF_P0_* and EKF_INITIAL_GYRO_BIAS.
 *  Called from odometry_reset() so a workstation "Reset odometry" command
 *  clears both pose and EKF state cleanly.
 * =============================================================================
 */

void   ekf_init(void);
void   ekf_reset(void);
void   ekf_tick(float dt_s);

float  ekf_theta(void);          /* fused heading, wrapped to [-π, π] */
float  ekf_omega(void);          /* best estimate of true angular velocity */
float  ekf_gyro_bias(void);      /* learned gyro bias (rad/s) */

bool   ekf_slip_detected(void);  /* true if last enc-ω update was rejected */
uint32_t ekf_slip_count(void);   /* cumulative count of rejected updates */

#endif /* EKF_H */
