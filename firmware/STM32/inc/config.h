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
#define DISABLE_PROXIMITY           0
#define DISABLE_LOAD_CELLS          0
#define DISABLE_IMU                 0
#define DISABLE_CURRENT_SENSE       0
#define DISABLE_ODOMETRY            0
#define DISABLE_OUTER_LOOP          0       /* navigators feed kinematic split directly when 1 */
#define DISABLE_TELEMETRY           0
#define DISABLE_LOG_FORWARDING      0       /* log entries still recorded; just not sent over UART */

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
#define TELEMETRY_RATE_MOVING_HZ    50u
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
#define WHEEL_RADIUS_M              0.0500f
#define WHEEL_BASE_M                0.2400f
#define ENCODER_PPR                 600u    /* OMRON E6B2 quadrature pulses per revolution */
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

/* Inner loop: per-wheel velocity PID. Independent gains per side to compensate
 * for mechanical asymmetry (different bearings, motor binding, etc.). */
#define PID_INNER_KP_LEFT           80.0f
#define PID_INNER_KI_LEFT           200.0f
#define PID_INNER_KD_LEFT           0.0f
#define PID_INNER_KP_RIGHT          80.0f
#define PID_INNER_KI_RIGHT          200.0f
#define PID_INNER_KD_RIGHT          0.0f

/* Outer loop: chassis-level (linear, angular) feedback against odometry. */
#define PID_OUTER_LIN_KP            1.0f
#define PID_OUTER_LIN_KI            0.5f
#define PID_OUTER_LIN_KD            0.0f
#define PID_OUTER_ANG_KP            1.0f
#define PID_OUTER_ANG_KI            0.5f
#define PID_OUTER_ANG_KD            0.0f

#define PID_OUTPUT_MIN              -1.0f   /* normalised duty; sign = direction */
#define PID_OUTPUT_MAX              1.0f
#define PID_INTEGRAL_LIMIT          1.0f

/* Navigator-internal control gains (separate from cascade). */
#define LINE_FOLLOW_CRUISE_MPS      0.3f
#define LINE_FOLLOW_KP              4.0f
#define LINE_FOLLOW_KI              0.0f
#define LINE_FOLLOW_KD              0.2f

#define PURE_PURSUIT_LOOKAHEAD_M    0.20f
#define TRAJECTORY_CRUISE_MPS       0.3f

/* ---- PWM ------------------------------------------------------------------ */
#define PWM_FREQ_HZ                 20000u  /* 20 kHz: above audible, fits Pololu G2 spec */
#define PWM_RESOLUTION              (SYSCLK_HZ / PWM_FREQ_HZ)   /* TIM1 ARR */

/* ---- Cargo / load cells (per-corner: FL, FR, RL, RR) ---------------------- */
#define WEIGHT_TOTAL_CAUTION_KG     80.0f
#define WEIGHT_TOTAL_ESTOP_KG       100.0f
#define WEIGHT_IMBALANCE_CAUTION    0.20f   /* fraction of total: corner deviation > this → CAUTION */
#define WEIGHT_IMBALANCE_ESTOP      0.40f   /* > this → CRITICAL or E-STOP */
#define HX711_DEFAULT_SCALE         1.0f    /* counts → kg, per cell, calibrated at runtime */
#define HX711_DEFAULT_OFFSET        0       /* tare offset in raw counts */
#define HX711_TIMEOUT_US            200u    /* if DOUT not low within this window, log timeout */

/* ---- Caution modifier levels --------------------------------------------- */
#define CAUTION_NORMAL              1.0f
#define CAUTION_LEVEL_CAUTION       0.5f
#define CAUTION_LEVEL_CRITICAL      0.2f

/* ---- Motor current sense (Pololu G2: ~20 mV/A; 3.3V / 4096 → ~40.3 mA/count) */
#define MOTOR_CURRENT_MA_PER_COUNT_NUM  40283u
#define MOTOR_CURRENT_MA_PER_COUNT_DEN  1000u
#define MOTOR_OVERCURRENT_MA            10000u  /* trip threshold per motor */

/* ---- Trajectory ----------------------------------------------------------- */
#define MAX_WAYPOINTS               64u
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
#define FAULT_LOG_DEPTH             64u     /* RAM ring buffer entries */

/* ---- ACK retry (workstation-side: STM32 only ACKs, doesn't retry) --------- */
#define ACK_TIMEOUT_MS              50u
#define ACK_MAX_RETRIES             3u

#endif /* CONFIG_H */
