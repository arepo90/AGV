#ifndef CONFIG_H
#define CONFIG_H

/* =============================================================================
 * AGV STM32F051 firmware configuration.
 *
 * Every tunable lives here. Anything wired or hard-coded outside this file is
 * dictated by silicon (pin → peripheral mappings) and is intentional.
 *
 * Layering: this header is included by both the bare-metal hal/ tier and the
 * app/ tier, so it must stay dependency-free (plain #defines only).
 * =============================================================================
 */

/* ---- Build-time toggles --------------------------------------------------- */
#define ENABLE_IWDG                 1       /* independent watchdog; disable for SWD step-debug */
#define IWDG_TIMEOUT_MS             500u    /* main loop must pet within this window */

/* ---- Module disable flags (bench bring-up) --------------------------------
 * Each flag, when 1, removes that subsystem's main-loop integration AND its
 * init. Default 0 = hardware present. Flip one to isolate a misbehaving module
 * without editing main.c. grep DISABLE_ to see the current build config. */
#define DISABLE_HEARTBEAT_WATCH     0
#define DISABLE_ESTOP               0
#define DISABLE_PROXIMITY           0
#define DISABLE_LOAD_CELLS          0
#define DISABLE_IMU                 1
#define DISABLE_TOF                 0       /* VL53L0X ranging via I2C mux */
#define DISABLE_BATTERY             0       /* INA219 3S/6S bus voltage */
#define DISABLE_CURRENT_SENSE       0
#define DISABLE_ODOMETRY            0
#define DISABLE_TELEMETRY           0
#define DISABLE_LOG_FORWARDING      0       /* still recorded, just not sent */
#define DISABLE_LIDAR               1       /* Jetson-segmented LaserScan (pushed over UART) */

/* ---- Clocks --------------------------------------------------------------- */
#define SYSCLK_HZ                   48000000u
#define PCLK_HZ                     48000000u
#define SYSTICK_HZ                  1000u    /* 1 kHz scheduler tick */

/* ---- USART1 link to ESP32 ------------------------------------------------- */
#define UART_BAUD                   921600u
#define UART_RX_RING_SIZE           512u     /* DMA circular RX buffer */
#define UART_TX_SLOTS               8u       /* outbound frame ring depth */

/* ---- Packet protocol ------------------------------------------------------ */
#define PROTO_MAGIC0                0xAAu
#define PROTO_MAGIC1                0x56u
#define PROTO_VERSION               0x03u    /* v3: +led_indicator_cfg in CORE, lidar tail in SENSORS, PKT_LIDAR_SEGMENTS */
#define PROTO_MAX_PAYLOAD           255u
#define PROTO_FRAME_OVERHEAD        8u       /* magic(2)+ver+seq+type+len+crc(2) */
#define PROTO_MAX_FRAME             (PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD)

/* ---- Heartbeat ------------------------------------------------------------ */
#define HEARTBEAT_TIMEOUT_MS        1000u    /* SUPERVISED → UNSUPERVISED */
#define HEARTBEAT_GRACE_MS          3000u    /* further wait before virtual E-STOP */

/* ---- Control loop --------------------------------------------------------- */
#define CONTROL_LOOP_HZ             100u     /* 100 Hz: ample for AGV dynamics, clean encoder vel */

/* ---- Telemetry stream rates ----------------------------------------------- */
/* Rate-grouped streams instead of one monolithic frame. Each is sent on its
 * own cadence so slow-changing data isn't re-packed at the fast rate. */
#define TLM_CORE_HZ_MOVING          50u      /* operational state + pose while navigating */
#define TLM_CORE_HZ_IDLE            10u      /* ...and while in STANDBY */
#define TLM_DRIVE_HZ                20u      /* per-wheel control internals (raise to tune PI) */
#define TLM_SENSORS_HZ              5u       /* load cells + IMU orientation */
/* QTR stream is sent at the control rate while LINE_FOLLOW or QTR-cal is active,
 * and not at all otherwise. */

/* ---- Sensor poll rates ---------------------------------------------------- */
#define ADC_SCAN_HZ                 100u     /* motor current + QTR multi-channel scan */
#define IMU_READ_HZ                 100u     /* MPU6050 gyro poll rate (internal DLPF runs at 1 kHz) */
#define HX711_RATE_STANDBY_HZ       30u      /* also weight-setting period (see UNVERIFIED) */
#define HX711_RATE_MOVING_HZ        2u

/* ---- Mechanical ----------------------------------------------------------- */
#define WHEEL_RADIUS_M              0.1f
#define WHEEL_BASE_M                0.4f
#define ENCODER_PPR                 500u     /* OMRON E6B2 pulses/rev (confirmed) */
#define ENCODER_QUADRATURE_FACTOR   4u       /* 4x decoding */
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE_FACTOR)
#define ENCODER_VEL_LPF_ALPHA       0.3f     /* v = α·new + (1-α)·prev; lower = smoother */

#define MOTOR_INVERT_LEFT           0
#define MOTOR_INVERT_RIGHT          0
#define ENCODER_INVERT_LEFT         1
#define ENCODER_INVERT_RIGHT        0

/* ---- Motion limits (defaults — runtime-tunable via PARAM_UPDATE) ---------- */
#define MAX_LINEAR_SPEED_MPS        1.0f
#define MAX_ANGULAR_SPEED_RADPS     2.0f
#define MAX_LINEAR_ACCEL_MPSS       0.8f
#define MAX_ANGULAR_ACCEL_RADPSS    2.0f

/* ---- Per-wheel velocity PI + feedforward ----------------------------------
 * duty = clamp( Kff·v_target + Kp·e + Ki·∫e dt , ±1 ).  No derivative term —
 * encoder velocity is too quantised to differentiate. Independent gains per
 * wheel absorb mechanical asymmetry. The feedforward carries the steady-state
 * (≈ duty needed per m/s ≈ 1/MAX_LINEAR_SPEED_MPS); the integrator only trims. */
#define WHEEL_KP_LEFT               1.0f
#define WHEEL_KI_LEFT               0.0f
#define WHEEL_KFF_LEFT              0.0f
#define WHEEL_KP_RIGHT              1.0f
#define WHEEL_KI_RIGHT              0.0f
#define WHEEL_KFF_RIGHT             0.0f
#define WHEEL_I_LIMIT               0.0f     /* integral clamp, in duty units */

/* ---- Line-follow PID (the only PID left; output is ω) --------------------- */
#define LINE_FOLLOW_CRUISE_MPS      0.3f
#define LINE_FOLLOW_KP              1.0f
#define LINE_FOLLOW_KI              0.0f
#define LINE_FOLLOW_KD              0.0f

/* ---- Trajectory (pure pursuit) -------------------------------------------- */
#define PURE_PURSUIT_LOOKAHEAD_M    0.50f
#define TRAJECTORY_CRUISE_MPS       0.3f
#define TRAJECTORY_CURV_SLOWDOWN    0.5f     /* v = cruise / (1 + g·|κ|); g in m */
#define MAX_WAYPOINTS               32u
#define WAYPOINT_REACH_RADIUS_M     0.05f

/* ---- QTR-8A reflectance (used when no flash calibration loaded) ----------- */
#define QTR_DEFAULT_WHITE           300u     /* counts on a white surface */
#define QTR_DEFAULT_BLACK           3000u    /* counts on a black line */
#define QTR_INVERT_ARRAY            0        /* 1 if sensor 7 is leftmost */
#define QTR_LINE_LOST_THRESHOLD     0.5f     /* sum-of-normalised below this → lost */

/* QTR_PIN_MAP — wiring remapping for analog_qtr(idx).
 * QTR_PIN_MAP[logical_idx] = ADC slot index (0..7), where each slot is a
 * fixed physical pin in channel-number order:
 *   slot 0 = PA4   slot 1 = PA5
 *   slot 2 = PC0   slot 3 = PC1   slot 4 = PC2   slot 5 = PC3
 *   slot 6 = PC4   slot 7 = PC5 */
#define QTR_PIN_MAP                 { 2, 3, 4, 5, 0, 1, 7, 6}

/* ---- Heading fusion (2-state Kalman filter: θ, gyro bias) -----------------
 * The MPU6050 has no magnetometer, so there is NO absolute heading reference;
 * yaw is integrated from the gyro and would drift unbounded. A 2-state linear
 * Kalman filter [θ, b] estimates and cancels the slowly-drifting gyro bias by
 * fusing two yaw-rate sources:
 *   - gyro Z          (slip-immune; biased)   → the prediction input
 *   - encoder diff ω  (bias-free; slip-prone)  → the measurement, gated off slip
 * Predict: θ += (gyro - b)·dt ; b held. Update: z observes true rate (gyro - b).
 * A zero-velocity update (ZUPT) feeds z = 0 whenever the robot is verified at
 * rest, pinning the bias each time it pauses — the practical drift bound with no
 * compass. Falls back to encoder-differential heading when the IMU is absent.
 * Position stays dead-reckoned (no sensor observes it yet); promote θ,x,y to a
 * full EKF when LiDAR/SLAM supplies an absolute fix. Tunables (rad, rad/s):
 *   Q_THETA  process noise on θ  (gyro white noise; larger → trust gyro less)
 *   Q_BIAS   bias random-walk    (larger → bias adapts faster, tracks temp drift)
 *   R_ENC    encoder yaw-rate measurement variance (larger → trust encoders less)
 *   R_ZUPT   zero-velocity measurement variance (small → snap bias hard at rest) */
#define HEADING_KF_Q_THETA          1.0e-4f
#define HEADING_KF_Q_BIAS           1.0e-7f
#define HEADING_KF_R_ENC            5.0e-3f
#define HEADING_KF_R_ZUPT           1.0e-5f
#define HEADING_KF_P0_THETA         1.0e-2f  /* initial θ variance */
#define HEADING_KF_P0_BIAS          1.0e-2f  /* initial bias variance (pre-convergence) */
#define HEADING_KF_BIAS_CONVERGED   1.0e-4f  /* bias variance below this → "converged" flag */
#define HEADING_SLIP_REJECT_RADPS   0.30f    /* |gyro - enc_ω| above this → skip enc update (slip) */
#define ZUPT_VEL_EPS_MPS            0.01f    /* both wheel speeds below this → candidate at-rest */
#define IMU_HEADING_SIGN            (-1.0f)  /* MPU6050 gyro-Z sign → encoder CCW convention */

/* ---- IMU tilt (accel+gyro complementary; diagnostic) ---------------------- */
#define IMU_TILT_COMP_ALPHA         0.98f    /* nearer 1 trusts the gyro; (1-α) pulls to gravity */

/* ---- PWM ------------------------------------------------------------------ */
#define PWM_FREQ_HZ                 20000u   /* 20 kHz: above audible, within Pololu G2 spec */
#define PWM_PERIOD                  (SYSCLK_HZ / PWM_FREQ_HZ)   /* TIM1 ARR+1 */

/* ---- Cargo / load cells (per-corner: FL, FR, RL, RR) ---------------------- */
#define HX711_NUM_CORNERS           4u
#define HX711_CORNER_FRONT_LEFT     0        /* PB12 */
#define HX711_CORNER_FRONT_RIGHT    1        /* PB13 */
#define HX711_CORNER_REAR_LEFT      2        /* PB14 */
#define HX711_CORNER_REAR_RIGHT     3        /* PB15 */
#define HX711_DEFAULT_SCALE         (5.7e-5f)  /* counts → kg per cell; calibrated at runtime */
#define HX711_DEFAULT_OFFSET        0           /* tare offset in raw counts */
#define HX711_TIMEOUT_MS            50u
#define WEIGHT_TOTAL_CAUTION_KG     80.0f
#define WEIGHT_TOTAL_ESTOP_KG       100.0f
#define WEIGHT_IMBALANCE_CAUTION    0.20f    /* corner deviation fraction → CAUTION */
#define WEIGHT_IMBALANCE_ESTOP      0.40f    /* → CRITICAL / E-STOP */

/* ---- Caution modifier levels ---------------------------------------------- */
#define CAUTION_NORMAL              1.0f
#define CAUTION_LEVEL_CAUTION       0.5f
#define CAUTION_LEVEL_CRITICAL      0.2f

/* ---- Indicator lights (ESP32 WS2812B rings) -------------------------------
 * The firmware only carries the animation style; the strips, counts, and colors
 * live on the ESP32. The GUI sets the style via PARAM_LED_MODE; it is reported
 * back in TLM_CORE so the ESP32 reads it from the telemetry tap. */
#define LED_MODE_PULSE              0u
#define LED_MODE_SNAKE              1u
#define LED_MODE_DEFAULT            LED_MODE_PULSE

/* The distance-reactive big ring carries two more operator settings, packed into
 * one byte (led_indicator_cfg) echoed in TLM_CORE alongside led_mode. The firmware
 * is just a pass-through for these — the ring logic itself lives on the ESP32. */
#define LED_IND_BASE_BIT            0u       /* 0 = base OFF, 1 = base WHITE */
#define LED_IND_MODE_BIT            1u       /* 0 = FIXED spread, 1 = RESPONSIVE spread */
#define LED_INDICATOR_CFG_DEFAULT   0u       /* base off, fixed */

/* ---- Proximity sensor logical mapping (EXTI line = bit position) ---------- */
#define PROX_FACING_FRONT           6u       /* PC6 */
#define PROX_FACING_REAR            7u       /* PC7 */
#define PROX_FACING_LEFT            8u       /* PC8 */
#define PROX_FACING_RIGHT           9u       /* PC9 */
#define PROX_ACTIVE_LOW             1        /* E18-D80NK NPN: pin low = obstacle */

/* ---- I2C bus device addresses (7-bit) -------------------------------------
 * All share I2C1 (PB8/PB9). The MPU6050 (0x68) and both INA219 sit directly on
 * the bus; the four VL53L0X share the default 0x29 and are isolated by the
 * TCA9548A mux (one channel exposed at a time), so they keep that address.
 * No collisions (0x68 vs 0x40/0x41/0x70). */
#define TCA9548A_I2C_ADDR           0x70u    /* I2C mux (A0..A2 low) */
#define VL53L0X_I2C_ADDR            0x29u    /* default; one live per mux channel */
#define INA219_3S_I2C_ADDR          0x40u    /* A0=A1=GND */
#define INA219_6S_I2C_ADDR          0x41u    /* A0=Vs, A1=GND */

/* ---- MPU6050 6-DOF IMU (gyro + accel; no magnetometer) --------------------
 * ±250 dps / ±2 g for best resolution on a slow ground robot. The DLPF is the
 * key knob in a vibrating metal enclosure: start at CFG 3 (gyro BW ≈ 44 Hz) and
 * tighten toward 5 (10 Hz) / 6 (5 Hz) if motor vibration shows in the gyro.
 * If FS_SEL/AFS_SEL change, update the matching LSB scale factors. */
#define MPU6050_I2C_ADDR            0x68u    /* AD0 low; 0x69 if high */
#define MPU6050_DLPF_CFG            3u       /* CONFIG[2:0]: 0/7 off, 1..6 = 188..5 Hz gyro BW */
#define MPU6050_SMPLRT_DIV          0u       /* sample rate = 1 kHz / (1+DIV); we subsample at IMU_READ_HZ */
#define MPU6050_GYRO_FS_SEL         0u       /* 0=±250, 1=±500, 2=±1000, 3=±2000 dps */
#define MPU6050_ACCEL_FS_SEL        0u       /* 0=±2, 1=±4, 2=±8, 3=±16 g */
#define MPU6050_GYRO_LSB_PER_DPS    131.0f   /* matches FS_SEL 0 (±250 dps) */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f /* matches AFS_SEL 0 (±2 g) */

/* ---- TOF distance sensors (VL53L0X ×4 behind the mux) ---------------------
 * One per corner, mirroring the IR proximity arrangement but giving graduated
 * distance bands. The MINIMUM distance across all present sensors selects a
 * caution level; below the E-STOP band it asserts an auto-clearing E-STOP, just
 * like the IR ring. All bands are runtime-tunable (PARAM_TOF_*); both caution
 * and E-STOP auto-clear with no workstation action. */
#define TOF_NUM_SENSORS             4u
#define TOF_MUX_CH_FRONT            0u       /* mux channel per facing */
#define TOF_MUX_CH_REAR             1u
#define TOF_MUX_CH_LEFT             2u
#define TOF_MUX_CH_RIGHT            3u
#define TOF_POLL_HZ                 60u      /* aggregate; round-robin one sensor/pass → TOF_POLL_HZ/N per sensor */
#define TOF_VALID_MAX_MM            1200u    /* readings beyond this (or status-fail) = "clear" */
#define TOF_CAUTION_MM              800u     /* < this → CAUTION (0.5) */
#define TOF_CRITICAL_MM             400u     /* < this → CRITICAL (0.2) */
#define TOF_ESTOP_MM                200u     /* < this → auto-clearing E-STOP */
#define TOF_BUDGET_US               33000u   /* VL53L0X measurement timing budget */

/* ---- LiDAR distance segments (Jetson-segmented LaserScan) ------------------
 * The 2D scan lives on the Jetson; lidar_node masks a hardcoded angular sector,
 * bins the rest into fixed angular intervals (average range per interval), and
 * pushes the per-interval distances (mm) down to the STM32 in PKT_LIDAR_SEGMENTS.
 * They are treated exactly like the TOF sensors: the MINIMUM fresh segment selects
 * a caution level and, below the E-STOP band, an auto-clearing E-STOP. The
 * segments are echoed back up in TLM_SENSORS so the ESP32 ring + GUI can show them.
 * Bands default to the TOF values and are runtime-tunable (PARAM_LIDAR_*). */
#define LIDAR_MAX_SEGMENTS          32u      /* wire/telemetry cap on interval count */
#define LIDAR_STALE_MS              500u     /* no fresh segments within this → treat as clear */
#define LIDAR_VALID_MAX_MM          8000u    /* "clear" sentinel; empty/absent bins report this */
#define LIDAR_CAUTION_MM            800u     /* < this (min over segments) → CAUTION (0.5) */
#define LIDAR_CRITICAL_MM           400u     /* < this → CRITICAL (0.2) */
#define LIDAR_ESTOP_MM              200u     /* < this → auto-clearing E-STOP */

/* ---- Battery monitor (INA219 bus voltage only — no current sense) ----------
 * Both modules read bus voltage (LSB 4 mV, BRNG=1 → 32 V FSR). The 3S rail
 * (motors + logic) gets caution then auto-clearing E-STOP on undervoltage, with
 * a recover margin so sag-under-load can't chatter. The 6S rail powers only the
 * Jetson — the STM32 cannot protect it and throttling would not help — so it is
 * display + a log warning only. Thresholds are per-pack totals (mV). */
#define INA219_BUS_LSB_UV           4000u    /* 4 mV per bit (BRNG=1, after >>3) */
#define BATTERY_POLL_HZ             2u
#define BATTERY_3S_CAUTION_MV       10500u   /* ≈3.50 V/cell → CAUTION */
#define BATTERY_3S_ESTOP_MV         9900u    /* ≈3.30 V/cell → E-STOP */
#define BATTERY_RECOVER_MV          300u     /* hysteresis: clear only above trip + this */
#define BATTERY_6S_WARN_MV          19800u   /* ≈3.30 V/cell → log warning (display rail) */
#define BATTERY_MIN_VALID_MV        3000u    /* below this assume sensor absent, not flat */

/* ---- Fault log ------------------------------------------------------------ */
#define FAULT_LOG_DEPTH             16u      /* RAM ring; drained every main loop */

/* =============================================================================
 *  UNVERIFIED — confirm against the physical board, then remove this banner.
 *  Carried over from the previous firmware; NOT checked on this hardware.
 *  Grouped here so they are trivial to correct in one place.
 * =============================================================================
 *  - HX711: 30 Hz standby reads require the chip in 80 SPS mode (RATE pin HIGH).
 *    If RATE is tied LOW (10 SPS default), set HX711_RATE_STANDBY_HZ <= 10.
 *  - Motor current scale: the old comment claimed ~40.3 mA/count but the
 *    constant is ~1.611 mA/count (≈25x apart). Verify with a known load before
 *    trusting the overcurrent trip threshold.
 */
#define MOTOR_CURRENT_MA_PER_COUNT_NUM  1611u
#define MOTOR_CURRENT_MA_PER_COUNT_DEN  1000u
#define MOTOR_OVERCURRENT_MA            10000u   /* per-motor trip threshold */

#endif /* CONFIG_H */
