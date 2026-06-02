#ifndef CONFIG_H
#define CONFIG_H

/* =============================================================================
 * AGV STM32 firmware configuration
 *
 * Every tunable lives here. Anything that is wired or hard-coded outside this
 * file is intentional (e.g. pin → peripheral mappings dictated by silicon).
 * =============================================================================
 */

/* ---- Build-time toggles --------------------------------------------------- */
#define ENABLE_IWDG                 1       /* 1 = enable independent watchdog. Disable for SWD step-debug. */
#define IWDG_TIMEOUT_MS             500     /* watchdog window. main loop must pet IWDG within this. */

/* ---- Module disable flags (for bench debugging) --------------------------- */
/* Each flag, when set to 1, removes that subsystem's main-loop integration so
 * the rest of the firmware runs without it. Init code is also skipped. Useful
 * for isolating which module is misbehaving without rewriting main.c. */
#define DISABLE_HEARTBEAT_WATCH     0
#define DISABLE_ESTOP               0
#define DISABLE_PROXIMITY           1
#define DISABLE_LOAD_CELLS          1
#define DISABLE_IMU                 1
#define DISABLE_CURRENT_SENSE       0
#define DISABLE_ODOMETRY            0
#define DISABLE_OUTER_LOOP          1       /* debug toggle. control.c gates the outer loop on
                                             * (USE_OUTER_CASCADE && !DISABLE_OUTER_LOOP) — keep
                                             * USE_OUTER_CASCADE as the architectural switch and
                                             * use this flag for short-term bring-up bypasses. */
#define DISABLE_TELEMETRY           0
#define DISABLE_LOG_FORWARDING      0       /* log entries still recorded; just not sent over UART */
#define DISABLE_HEADING_FUSION      1       /* 1 = pure encoder odometry (ignore IMU yaw) */

/* ---- Clocks --------------------------------------------------------------- */
#define SYSCLK_HZ                   48000000u
#define PCLK_HZ                     48000000u
#define SYSTICK_HZ                  1000u    /* 1 kHz scheduler tick */

/* ---- USART1 link to ESP32 ------------------------------------------------- */
#define UART_BAUD                   921600u
#define UART_RX_RING_SIZE           512u    /* DMA circular RX buffer (power of 2 not required) */
#define UART_TX_QUEUE_SIZE          512u    /* outbound queue for telemetry+log+ack */

/* ---- Packet protocol ------------------------------------------------------ */
#define PROTO_MAGIC0                0xAAu
#define PROTO_MAGIC1                0x56u
#define PROTO_VERSION               0x01u   /* version byte after MAGIC, see comms.h */
#define PROTO_MAX_PAYLOAD           255u
#define PROTO_FRAME_OVERHEAD        8u      /* magic(2)+ver(1)+seq(1)+type(1)+len(1)+crc(2) */
#define PROTO_MAX_FRAME             (PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD)

/* ---- Heartbeat ------------------------------------------------------------ */
#define HEARTBEAT_TIMEOUT_MS        1000u   /* SUPERVISED → UNSUPERVISED on this much silence */
#define HEARTBEAT_GRACE_MS          3000u   /* further wait before virtual E-STOP fires */

/* ---- Telemetry ------------------------------------------------------------ */
#define TELEMETRY_RATE_MOVING_HZ    20u
#define TELEMETRY_RATE_STANDBY_HZ   5u

/* ---- Control loop --------------------------------------------------------- */
#define CONTROL_LOOP_HZ             200u

/* ---- Sensor poll rates ---------------------------------------------------- */
#define ADC_SCAN_HZ                 100u    /* current sense; QTR8A piggybacks here when not LINE_FOLLOW */
#define QTR_LINE_FOLLOW_HZ          500u    /* dedicated faster scan during LINE_FOLLOW */
#define IMU_READ_HZ                 100u
#define HX711_RATE_STANDBY_HZ       30u     /* also used during weight-setting period */
#define HX711_RATE_MOVING_HZ        2u

/* ---- Mechanical ----------------------------------------------------------- */
#define WHEEL_RADIUS_M              0.1f
#define WHEEL_BASE_M                0.2f
#define ENCODER_PPR                 500u    /* OMRON E6B2 quadrature pulses per revolution */
#define ENCODER_QUADRATURE_FACTOR   4u      /* 4x decoding (TI_TI_RISING_FALLING) */
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE_FACTOR)

#define MOTOR_INVERT_LEFT           0
#define MOTOR_INVERT_RIGHT          0
#define ENCODER_INVERT_LEFT         0
#define ENCODER_INVERT_RIGHT        0

/* ---- Motion limits (defaults — runtime-tunable via PARAM_UPDATE) ---------- */
#define MAX_LINEAR_SPEED_MPS        1.0f
#define MAX_ANGULAR_SPEED_RADPS     2.0f
#define MAX_LINEAR_ACCEL_MPSS       0.8f
#define MAX_ANGULAR_ACCEL_RADPSS    2.0f

/* ---- Cascade controller --------------------------------------------------- */
#define USE_OUTER_CASCADE           1       /* 0 = navigator → kinematic split → inner only */

/* Super-twisting sliding-mode controllers (Levant) replace the cascade PIDs.
 *   u̇₁ = k₂·sign(s);   u = k₁·√|s|·sign(s) + u₁;   s = setpoint − measurement
 *
 * Two gains per loop: k1 (continuous, like Kp on √|s|) and k2 (discontinuous,
 * the integrator that compensates matched disturbances). u1 is clamped to the
 * loop's output magnitude as anti-windup. */

/* Inner loop: per-wheel velocity STA. Independent gains per side to compensate
 * for mechanical asymmetry (different bearings, motor binding, etc.). Output
 * is normalised duty in [-1, +1]. */
#define STA_INNER_K1_LEFT           1.0f
#define STA_INNER_K2_LEFT           0.0f
#define STA_INNER_K1_RIGHT          1.0f
#define STA_INNER_K2_RIGHT          0.0f

/* Outer loop: chassis-level (linear, angular) STA against odometry. Output is
 * a corrected (v, ω) setpoint, clamped to the chassis maxima. */
#define STA_OUTER_LIN_K1            1.0f
#define STA_OUTER_LIN_K2            0.5f
#define STA_OUTER_ANG_K1            1.0f
#define STA_OUTER_ANG_K2            0.5f

#define STA_INNER_OUTPUT_MIN        -1.0f   /* normalised duty; sign = direction */
#define STA_INNER_OUTPUT_MAX        1.0f
#define STA_INNER_U1_LIMIT          1.0f
/* Outer u1 bounds use the chassis maxima — set in control.c, not macroed. */

/* Navigator-internal control gains (separate from cascade). */
#define LINE_FOLLOW_CRUISE_MPS      0.3f
#define LINE_FOLLOW_KP              1.0f
#define LINE_FOLLOW_KI              0.0f
#define LINE_FOLLOW_KD              0.0f

#define PURE_PURSUIT_LOOKAHEAD_M    0.50f
#define TRAJECTORY_CRUISE_MPS       0.3f
#define TRAJECTORY_CURV_SLOWDOWN    0.5f    /* m, in v = cruise / (1 + g·|κ|). g·|κ| is
                                             * unitless (κ is 1/m). g=0.5 halves v at a
                                             * 0.5 m turn radius (|κ|=2 m⁻¹). 0 disables. */

/* QTR-8A reflectance baselines (used when no flash calibration loaded yet).
 * White surface ≈ low reading, black surface ≈ high reading on QTR-8A. */
#define QTR_DEFAULT_WHITE           300u    /* counts on a white surface */
#define QTR_DEFAULT_BLACK           3000u   /* counts on a black line */
#define QTR_INVERT_ARRAY            0       /* 1 if sensor 7 is leftmost, not 0 */
#define QTR_LINE_LOST_THRESHOLD     0.5f    /* sum-of-normalized below this → line lost */

/* ---- EKF heading fusion --------------------------------------------------
 * 2-state EKF: x = [θ, b_g]ᵀ.
 *   Predict: θ̇ = ω_imu - b_g  (gyro is the input; bias is a random walk)
 *   Measure: BNO055 absolute yaw → corrects θ
 *            Encoder ω           → corrects (ω_imu - b_g), learns bias,
 *                                  rejects outliers (wheel slip) via chi²
 *
 * Tuning intuition:
 *   - Bigger Q  → trust prediction less, react faster to measurements (more noise)
 *   - Bigger R  → trust this measurement less (slower correction, smoother)
 *   - Bigger P0 → initial covariance large = updates land harder for first few ticks
 *   - SLIP_GATE_N is in σ-units; |y| > N·√S rejects the encoder update and logs.
 *     N=3 ≈ 99.7%-CL gate. Lower to reject slip more aggressively.
 *
 * Units: rad, rad/s, and their squares (variances).
 */

/* BNO055 yaw and gyro_z are returned in BNO055's body frame (default
 * "Windows" orientation: CW-positive looking down from above). Our encoder
 * convention is CCW-positive. Set to -1.0f to negate; +1.0f if your mount
 * already matches. Applied inside imu_yaw_rad() and imu_gyro_z_radps(). */
#define IMU_HEADING_SIGN            (-1.0f)

/* Process noise (per √second). Q_THETA reflects gyro white-noise std dev.
 * BNO055 gyro typical noise ~0.014 rad/s/√Hz → ~0.014²·100 ≈ 0.02 rad²/s at
 * 100 Hz. Start coarse, tighten after observation. */
#define EKF_Q_THETA                 0.001f   /* rad²/s  — gyro propagation noise */
#define EKF_Q_BIAS                  1e-7f    /* rad²/s² — gyro-bias random walk */

/* Measurement noise. BNO055 absolute yaw typically ±1° = 0.017 rad → σ²≈3e-4.
 * Encoder ω noise depends on PPR and dt — ~0.05 rad/s std on this rig. */
#define EKF_R_YAW                   3e-4f    /* rad²   — BNO055 yaw variance */
#define EKF_R_OMEGA_ENC             2.5e-3f  /* (rad/s)² — encoder ω variance */

/* Initial covariance (P diagonal). Set high so the first few measurements
 * dominate — the integrator settles in <1 s. */
#define EKF_P0_THETA                1.0f     /* rad²   — initial θ uncertainty */
#define EKF_P0_BIAS                 0.1f     /* (rad/s)² — initial bias uncertainty */
#define EKF_INITIAL_GYRO_BIAS       0.0f     /* rad/s   — seed bias estimate */

/* Outlier gate on encoder-ω measurement. If normalised innovation² > N²,
 * reject (suspect wheel slip). N=3 ≈ 99.7%-CL χ². */
#define EKF_SLIP_GATE_N             3.0f

/* Calibration gates: skip the relevant measurement if BNO055 has not warmed
 * up. NDOF yaw needs mag calibration; gyro needs gyro calibration. */
#define EKF_MIN_GYRO_CALIB          1u      /* gyro register usable */
#define EKF_MIN_YAW_CALIB           2u      /* NDOF yaw trustworthy */

/* ---- PWM ------------------------------------------------------------------ */
#define PWM_FREQ_HZ                 20000u  /* 20 kHz: above audible, fits Pololu G2 spec */
#define PWM_RESOLUTION              (SYSCLK_HZ / PWM_FREQ_HZ)   /* TIM1 ARR */

/* ---- Cargo / load cells (per-corner: FL, FR, RL, RR) ---------------------- */
#define WEIGHT_TOTAL_CAUTION_KG     80.0f
#define WEIGHT_TOTAL_ESTOP_KG       100.0f
#define WEIGHT_IMBALANCE_CAUTION    0.20f   /* fraction of total: corner deviation > this → CAUTION */
#define WEIGHT_IMBALANCE_ESTOP      0.40f   /* > this → CRITICAL or E-STOP */
#define HX711_DEFAULT_SCALE         -5.7e-5f    /* counts → kg, per cell, calibrated at runtime */
#define HX711_DEFAULT_OFFSET        0       /* tare offset in raw counts */
#define HX711_TIMEOUT_MS            50u     /* DOUT not low for this long after a scheduled read → log */

/* ---- Caution modifier levels --------------------------------------------- */
#define CAUTION_NORMAL              1.0f
#define CAUTION_LEVEL_CAUTION       0.5f
#define CAUTION_LEVEL_CRITICAL      0.2f

/* ---- Motor current sense (Pololu G2: ~20 mV/A; 3.3V / 4096 → ~40.3 mA/count) */
#define MOTOR_CURRENT_MA_PER_COUNT_NUM  1611u
#define MOTOR_CURRENT_MA_PER_COUNT_DEN  1000u
#define MOTOR_OVERCURRENT_MA            10000u  /* trip threshold per motor */

/* ---- Trajectory ----------------------------------------------------------- */
#define MAX_WAYPOINTS               32u     /* 32 × 8 B = 256 B; PKT_FRAG-bounded anyway */
#define WAYPOINT_REACH_RADIUS_M     0.05f

/* ---- BNO055 IMU ----------------------------------------------------------- */
#define BNO055_I2C_ADDR             0x28u   /* ADR pin low; 0x29 if high */
#define BNO055_OPMODE_NDOF          0x0Cu

/* ---- Proximity sensor logical mapping (EXTI source bit → facing) ---------- */
/* Pins are PC6/PC7/PC8/PC9 = EXTI lines 6/7/8/9. Source bitmask uses these
 * indices directly so PROX_FACING_FRONT etc. are the bit positions. */
#define PROX_FACING_FRONT           6u      /* PC6 */
#define PROX_FACING_REAR            7u      /* PC7 */
#define PROX_FACING_LEFT            8u      /* PC8 */
#define PROX_FACING_RIGHT           9u      /* PC9 */
#define PROX_ACTIVE_LOW             1       /* E18-D80NK NPN: pin low = obstacle */

/* ---- HX711 corner logical mapping (PB12-15 to physical position) ---------- */
#define HX711_CORNER_FRONT_LEFT     0       /* PB12 */
#define HX711_CORNER_FRONT_RIGHT    1       /* PB13 */
#define HX711_CORNER_REAR_LEFT      2       /* PB14 */
#define HX711_CORNER_REAR_RIGHT     3       /* PB15 */

/* ---- Fault log ------------------------------------------------------------ */
#define FAULT_LOG_DEPTH             16u     /* RAM ring buffer entries. Drained every main-loop
                                             * iteration (~kHz) so depth only matters during
                                             * UART-disconnect windows; overflows increment
                                             * log_dropped (visible in telemetry). */

/* ---- ACK retry (workstation-side: STM32 only ACKs, doesn't retry) --------- */
#define ACK_TIMEOUT_MS              50u
#define ACK_MAX_RETRIES             3u

#endif /* CONFIG_H */
