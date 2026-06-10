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
#define DISABLE_BATTERY             0       /* INA219 3S bus voltage */
#define DISABLE_CURRENT_SENSE       0
#define DISABLE_ODOMETRY            0
#define DISABLE_TELEMETRY           0
#define DISABLE_LOG_FORWARDING      0       /* still recorded, just not sent */
#define DISABLE_LIDAR               1       /* Jetson-segmented LaserScan (pushed over UART) */

/* Bench-only: 1 = boot straight into the I2C bus scanner (app/i2cscan), which
 * prints bus health + an address scan over USART1 and never returns. Leave 0
 * for normal firmware. */
#define I2C_SCAN                    0

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
#define PROTO_VERSION               0x04u    /* v4: TOF + 6S battery removed (SENSORS = 18B fixed + lidar tail) */
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
#define TLM_SENSORS_HZ              5u       /* load cells + battery + LiDAR echo */
/* QTR stream is sent at the control rate while LINE_FOLLOW is active, and not at
 * all otherwise. */

/* ---- Sensor poll rates ---------------------------------------------------- */
#define ADC_SCAN_HZ                 100u     /* motor current + QTR multi-channel scan */
#define HX711_RATE_STANDBY_HZ       30u      /* also weight-setting period (see UNVERIFIED) */
#define HX711_RATE_MOVING_HZ        2u

/* ---- Mechanical ----------------------------------------------------------- */
#define WHEEL_RADIUS_M              0.1f
#define WHEEL_BASE_M                0.4f
#define ENCODER_PPR                 500u     /* OMRON E6B2 pulses/rev (confirmed) */
#define ENCODER_QUADRATURE_FACTOR   4u       /* 4x decoding */
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE_FACTOR)
#define ENCODER_VEL_LPF_ALPHA       0.3f     /* v = α·new + (1-α)·prev; lower = smoother */

/* ---- Drive channel mapping -------------------------------------------------
 * Logical sides (LEFT/RIGHT) map to hardware channels here instead of being
 * hard-coded in control/encoders. Hardware channel 0 = TIM1_CH1 PWM (PA8) +
 * DIR PB0; channel 1 = TIM1_CH4 PWM (PA11) + DIR PB2. Encoder timer 0 = TIM2
 * (PA0/PA1); timer 1 = TIM3 (PA6/PA7).
 *
 * The defaults below reproduce the bench-verified arrangement that previously
 * lived as a hard-coded swap in control.c/encoders.c (sides crossed, left
 * chain inverted). UNVERIFIED against physical left/right — correct these four
 * pairs after a drive test (fwd cmd → both wheels forward; +ω → CCW). */
#define MOTOR_CH_LEFT               1
#define MOTOR_CH_RIGHT              0
#define MOTOR_INVERT_LEFT           1
#define MOTOR_INVERT_RIGHT          0
#define ENCODER_TIM_LEFT            1
#define ENCODER_TIM_RIGHT           0
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
#define WHEEL_KP_LEFT               2.0f
#define WHEEL_KI_LEFT               0.0f
#define WHEEL_KFF_LEFT              0.0f
#define WHEEL_KP_RIGHT              2.0f
#define WHEEL_KI_RIGHT              0.0f
#define WHEEL_KFF_RIGHT             0.0f
#define WHEEL_I_LIMIT               0.8f     /* raw ∫e·dt clamp (pre-Ki): duty contribution ≤ Ki × this */

/* ---- Line-follow PID (the only PID left; output is ω) --------------------- */
#define LINE_FOLLOW_CRUISE_MPS      0.2f
#define LINE_FOLLOW_KP              3.0f
#define LINE_FOLLOW_KI              0.0f
#define LINE_FOLLOW_KD              0.0f

/* ---- T-junction turnaround -------------------------------------------------
 * A wide perpendicular bar at the end of the line ("T") blacks out all/most of
 * the array; the AGV then turns 180° on its own axis and follows the line back.
 * The turn is odometry-gated (encoder heading), not timed, so it self-adjusts
 * to the caution modifier and battery sag — it therefore needs ODOMETRY enabled
 * (with it disabled, the turn times out into LINE_LOST). Detection is absolute:
 * the auto-ranging centroid can't tell all-black from all-white, so a sensor
 * counts as "black" above LINE_T_BLACK_COUNTS (runtime-tunable, PARAM 0x27). */
#define LINE_T_BLACK_COUNTS         4090    /* ADC counts (dark reads high); tune on the bench */
#define LINE_T_MIN_SENSORS          6u       /* ≥ this many black sensors = T bar */
#define LINE_T_DEBOUNCE_TICKS       3u       /* consecutive control frames before triggering */
#define LINE_TURN_CCW               1        /* 1 = turn left at the T, 0 = right */
#define LINE_TURN_OMEGA_RADPS       1.0f     /* on-axis turn rate (pre caution clamp) */
#define LINE_TURN_BLIND_RAD         1f    /* ~150°: minimum sweep before looking for the line */
#define LINE_TURN_MAX_RAD           6f    /* ~330° swept without a line → give up (LINE_LOST) */
#define LINE_TURN_TIMEOUT_MS        8000u    /* hard cap (covers frozen odometry) */

/* ---- QTR-8A line sensor (per-frame auto-ranging; no calibration) ---------- */
#define QTR_INVERT_ARRAY            0        /* 1 if sensor 7 is leftmost */
#define QTR_MIN_CONTRAST_COUNTS     100u       /* line lost if (max-min) across the array < this many ADC counts (0 disables) */

/* QTR_PIN_MAP — wiring remapping for analog_qtr(idx).
 * QTR_PIN_MAP[logical_idx] = ADC slot index (0..7), where each slot is a
 * fixed physical pin in channel-number order:
 *   slot 0 = PA4   slot 1 = PA5
 *   slot 2 = PC0   slot 3 = PC1   slot 4 = PC2   slot 5 = PC3
 *   slot 6 = PC4   slot 7 = PC5 */
#define QTR_PIN_MAP                 { 2, 3, 4, 5, 0, 1, 7, 6}

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
#define IMBALANCE_FLOOR_KG          5.0f     /* below this total, imbalance (caution + E-STOP) is noise — ignored */

/* ---- Caution modifier levels ---------------------------------------------- */
#define CAUTION_NORMAL              1.0f
#define CAUTION_LEVEL_CAUTION       0.5f
#define CAUTION_LEVEL_CRITICAL      0.2f
/* A workstation OVERRIDE_CAUTION has full authority for this long after the
 * last command, then releases back to the firmware's own per-source minimum
 * (so a stale override from a dead workstation can't pin the modifier). */
#define CAUTION_WS_OVERRIDE_TIMEOUT_MS  10000u

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
 * The I2C1 bus (PB8/PB9) is currently unused — the INA219 moved to the ESP32.
 * The HAL, scanner, and address stay for future expansion / moving it back. */
#define INA219_3S_I2C_ADDR          0x40u    /* A0=A1=GND */

/* ---- LiDAR distance segments (Jetson-segmented LaserScan) ------------------
 * The 2D scan lives on the Jetson; lidar_node masks a hardcoded angular sector,
 * bins the rest into fixed angular intervals (average range per interval), and
 * pushes the per-interval distances (mm) down to the STM32 in PKT_LIDAR_SEGMENTS.
 * The MINIMUM fresh segment selects a caution level and, below the E-STOP band,
 * an auto-clearing E-STOP. The segments are echoed back up in TLM_SENSORS so the
 * ESP32 ring + GUI can show them. Bands are runtime-tunable (PARAM_LIDAR_*). */
#define LIDAR_MAX_SEGMENTS          32u      /* wire/telemetry cap on interval count */
#define LIDAR_STALE_MS              500u     /* no fresh segments within this → treat as clear */
#define LIDAR_VALID_MAX_MM          8000u    /* "clear" sentinel; empty/absent bins report this */
#define LIDAR_CAUTION_MM            800u     /* < this (min over segments) → CAUTION (0.5) */
#define LIDAR_CRITICAL_MM           400u     /* < this → CRITICAL (0.2) */
#define LIDAR_ESTOP_MM              200u     /* < this → auto-clearing E-STOP */

/* ---- Battery monitor (INA219 bus voltage only — no current sense) ----------
 * One INA219 reads 3S bus voltage (LSB 4 mV, BRNG=1 → 32 V FSR). It hangs off
 * the ESP32's I2C; the ESP32 polls it and pushes each reading as PKT_BATTERY.
 * The 3S rail (motors + logic) gets caution then auto-clearing E-STOP on
 * undervoltage, with a recover margin so sag-under-load can't chatter.
 * Thresholds are per-pack totals (mV). The 6S Jetson rail is not monitored.
 * BATTERY_VIA_STM32_I2C = 1 revives the legacy direct driver on I2C1. */
#define BATTERY_VIA_STM32_I2C       0
#define BATTERY_PUSH_STALE_MS       2500u    /* no push within this → battery absent */
#define INA219_BUS_LSB_UV           4000u    /* 4 mV per bit (BRNG=1, after >>3) */
#define BATTERY_POLL_HZ             2u       /* legacy I2C path only; ESP32 mirrors it */
#define BATTERY_3S_CAUTION_MV       10500u   /* ≈3.50 V/cell → CAUTION */
#define BATTERY_3S_ESTOP_MV         9900u    /* ≈3.30 V/cell → E-STOP */
#define BATTERY_RECOVER_MV          300u     /* hysteresis: clear only above trip + this */
#define BATTERY_MIN_VALID_MV        3000u    /* below this assume sensor absent, not flat */

/* ---- Fault log ------------------------------------------------------------ */
#define FAULT_LOG_DEPTH             16u      /* RAM ring; drained every main loop */

/* =============================================================================
 *  UNVERIFIED — confirm against the physical board, then remove this banner.
 *  Carried over from the previous firmware; NOT checked on this hardware.
 *  Grouped here so they are trivial to correct in one place.
 * =============================================================================
 *  - Motor current scale: the old comment claimed ~40.3 mA/count but the
 *    constant is ~1.611 mA/count (≈25x apart). Verify with a known load before
 *    trusting the overcurrent trip threshold.
 */
#define MOTOR_CURRENT_MA_PER_COUNT_NUM  1611u
#define MOTOR_CURRENT_MA_PER_COUNT_DEN  1000u
#define MOTOR_OVERCURRENT_MA            10000u   /* per-motor trip threshold */

#endif /* CONFIG_H */
