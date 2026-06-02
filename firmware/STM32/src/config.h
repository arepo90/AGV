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
#define DISABLE_LOAD_CELLS          1
#define DISABLE_IMU                 1
#define DISABLE_TOF                 0       /* VL53L0X ranging via I2C mux */
#define DISABLE_BATTERY             0       /* INA219 3S/6S bus voltage */
#define DISABLE_CURRENT_SENSE       0
#define DISABLE_ODOMETRY            0
#define DISABLE_TELEMETRY           0
#define DISABLE_LOG_FORWARDING      0       /* still recorded, just not sent */

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
#define PROTO_VERSION               0x02u    /* v2: streamed telemetry + PI gains */
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
#define IMU_READ_HZ                 100u     /* BNO055 NDOF fusion caps at 100 Hz */
#define HX711_RATE_STANDBY_HZ       30u      /* also weight-setting period (see UNVERIFIED) */
#define HX711_RATE_MOVING_HZ        2u

/* ---- Mechanical ----------------------------------------------------------- */
#define WHEEL_RADIUS_M              0.1f
#define WHEEL_BASE_M                0.2f
#define ENCODER_PPR                 500u     /* OMRON E6B2 pulses/rev (confirmed) */
#define ENCODER_QUADRATURE_FACTOR   4u       /* 4x decoding */
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE_FACTOR)
#define ENCODER_VEL_LPF_ALPHA       0.3f     /* v = α·new + (1-α)·prev; lower = smoother */

#define MOTOR_INVERT_LEFT           0
#define MOTOR_INVERT_RIGHT          0
#define ENCODER_INVERT_LEFT         0
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
#define WHEEL_KP_LEFT               0.5f
#define WHEEL_KI_LEFT               2.0f
#define WHEEL_KFF_LEFT              1.0f
#define WHEEL_KP_RIGHT              0.5f
#define WHEEL_KI_RIGHT              2.0f
#define WHEEL_KFF_RIGHT             1.0f
#define WHEEL_I_LIMIT               1.0f     /* integral clamp, in duty units */

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

/* ---- Heading fusion (complementary filter) --------------------------------
 * θ integrates the slip-immune BNO055 gyro; a slow pull toward the BNO055
 * absolute yaw bounds long-term drift. One knob: ALPHA in [0,1). Nearer 1
 * trusts the gyro more; (1-ALPHA) is the fraction of yaw error corrected each
 * tick. Falls back to encoder-differential heading when the IMU is absent. */
#define HEADING_COMP_ALPHA          0.98f
#define IMU_HEADING_SIGN            (-1.0f)  /* BNO055 CW-yaw → encoder CCW convention */
#define HEADING_MIN_GYRO_CALIB      1u       /* gate gyro propagation on this calib level */
#define HEADING_MIN_YAW_CALIB       2u       /* gate absolute-yaw correction on this level */

/* ---- PWM ------------------------------------------------------------------ */
#define PWM_FREQ_HZ                 20000u   /* 20 kHz: above audible, within Pololu G2 spec */
#define PWM_PERIOD                  (SYSCLK_HZ / PWM_FREQ_HZ)   /* TIM1 ARR+1 */

/* ---- Cargo / load cells (per-corner: FL, FR, RL, RR) ---------------------- */
#define HX711_NUM_CORNERS           4u
#define HX711_CORNER_FRONT_LEFT     0        /* PB12 */
#define HX711_CORNER_FRONT_RIGHT    1        /* PB13 */
#define HX711_CORNER_REAR_LEFT      2        /* PB14 */
#define HX711_CORNER_REAR_RIGHT     3        /* PB15 */
#define HX711_DEFAULT_SCALE         (-5.7e-5f)  /* counts → kg per cell; calibrated at runtime */
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

/* ---- Proximity sensor logical mapping (EXTI line = bit position) ---------- */
#define PROX_FACING_FRONT           6u       /* PC6 */
#define PROX_FACING_REAR            7u       /* PC7 */
#define PROX_FACING_LEFT            8u       /* PC8 */
#define PROX_FACING_RIGHT           9u       /* PC9 */
#define PROX_ACTIVE_LOW             1        /* E18-D80NK NPN: pin low = obstacle */

/* ---- I2C bus device addresses (7-bit) -------------------------------------
 * All share I2C1 (PB8/PB9). The BNO055 (0x28) and both INA219 sit directly on
 * the bus; the four VL53L0X share the default 0x29 and are isolated by the
 * TCA9548A mux (one channel exposed at a time), so they keep that address. */
#define TCA9548A_I2C_ADDR           0x70u    /* I2C mux (A0..A2 low) */
#define VL53L0X_I2C_ADDR            0x29u    /* default; one live per mux channel */
#define INA219_3S_I2C_ADDR          0x40u    /* A0=A1=GND */
#define INA219_6S_I2C_ADDR          0x41u    /* A0=Vs, A1=GND */

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
#define BNO055_I2C_ADDR                 0x28u    /* ADR low; 0x29 if high */
#define BNO055_OPMODE_NDOF              0x0Cu

#endif /* CONFIG_H */
