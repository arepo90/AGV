#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 * Shared enums / typedefs across the firmware.
 *
 * Wire-visible IDs (mode, function, packet types, log codes) are consumed by
 * the ESP32 tap / ROS bridge / GUI. Append, don't reorder.
 * =============================================================================
 */

/* ---- Operating mode ------------------------------------------------------- */
typedef enum {
    MODE_SUPERVISED   = 0x00u,
    MODE_UNSUPERVISED = 0x01u,
} agv_mode_t;

/* ---- Navigation function -------------------------------------------------- */
typedef enum {
    FUNC_STANDBY        = 0x00u,
    FUNC_REMOTE_CONTROL = 0x01u,
    FUNC_LINE_FOLLOW    = 0x02u,
} agv_function_t;

/* ---- Motor / encoder side ------------------------------------------------- */
typedef enum {
    SIDE_LEFT  = 0,
    SIDE_RIGHT = 1,
} side_t;

/* ---- Reset cause (captured at boot) --------------------------------------- */
typedef enum {
    RESET_CAUSE_UNKNOWN   = 0,
    RESET_CAUSE_POWER_ON  = 1,
    RESET_CAUSE_PIN       = 2,
    RESET_CAUSE_SOFTWARE  = 3,
    RESET_CAUSE_WATCHDOG  = 4,
    RESET_CAUSE_LOW_POWER = 5,
} reset_cause_t;

/* ---- Caution modifier sources (bitmask) -----------------------------------
 * Wire-exposed as a 16-bit field in TLM_CORE (room to grow past 8 sources). */
typedef enum {
    CAUTION_SRC_NONE               = 0,
    CAUTION_SRC_LOAD_OVERWEIGHT    = (1u << 0),
    CAUTION_SRC_LOAD_IMBALANCE     = (1u << 1),
    CAUTION_SRC_UNSUPERVISED_NAV   = (1u << 2),
    CAUTION_SRC_PROXIMITY_NEAR     = (1u << 3),
    CAUTION_SRC_WORKSTATION_FORCED = (1u << 4),
    /* bit 5 retired (was TOF_NEAR — VL53L0X hardware removed) */
    CAUTION_SRC_BATTERY_LOW        = (1u << 6),  /* 3S sag */
    CAUTION_SRC_LIDAR_NEAR         = (1u << 7),  /* Jetson-segmented LaserScan band */
} caution_source_t;

/* ---- Virtual E-STOP sources (bitmask) -------------------------------------
 * Wire-exposed as a 16-bit field in TLM_CORE; the byte filled at bit 7. */
typedef enum {
    ESTOP_SRC_NONE              = 0,
    ESTOP_SRC_PROXIMITY         = (1u << 0),  /* auto-clears */
    ESTOP_SRC_CARGO_OVERLOAD    = (1u << 1),  /* auto-clears */
    ESTOP_SRC_CARGO_IMBALANCE   = (1u << 2),  /* auto-clears */
    ESTOP_SRC_HEARTBEAT_TIMEOUT = (1u << 3),  /* needs explicit clear */
    ESTOP_SRC_WORKSTATION       = (1u << 4),  /* needs explicit clear */
    ESTOP_SRC_OVERCURRENT       = (1u << 5),  /* needs explicit clear */
    ESTOP_SRC_FIRMWARE_FAULT    = (1u << 6),  /* needs explicit clear */
    /* bit 7 retired (was TOF — VL53L0X hardware removed) */
    ESTOP_SRC_BATTERY_LOW       = (1u << 8),  /* 3S undervoltage; auto-clears (hysteresis) */
    ESTOP_SRC_LIDAR             = (1u << 9),  /* LaserScan segment close range; auto-clears */
} estop_source_t;

#define ESTOP_AUTOCLEAR_MASK                                          \
    (ESTOP_SRC_PROXIMITY | ESTOP_SRC_CARGO_OVERLOAD |                 \
     ESTOP_SRC_CARGO_IMBALANCE | ESTOP_SRC_BATTERY_LOW |              \
     ESTOP_SRC_LIDAR)

/* ---- Fault log severity --------------------------------------------------- */
typedef enum {
    LOG_SEV_INFO     = 0u,
    LOG_SEV_WARN     = 1u,
    LOG_SEV_ERROR    = 2u,
    LOG_SEV_CRITICAL = 3u,
} log_severity_t;

/* ---- Fault log source module ---------------------------------------------- */
typedef enum {
    LOG_MOD_SYSTEM    = 0u,
    LOG_MOD_COMMS     = 1u,
    LOG_MOD_MOTORS    = 2u,
    LOG_MOD_ENCODERS  = 3u,
    LOG_MOD_ADC       = 4u,
    LOG_MOD_HX711     = 5u,
    LOG_MOD_PROXIMITY = 7u,
    LOG_MOD_ESTOP     = 8u,
    LOG_MOD_HEARTBEAT = 9u,
    LOG_MOD_STATE     = 10u,
    LOG_MOD_NAV       = 11u,
    LOG_MOD_ODOMETRY  = 12u,
    /* 13 retired (was TOF) */
    LOG_MOD_BATTERY   = 14u,
    LOG_MOD_LIDAR     = 15u,
} log_module_t;

/* ---- Fault log codes (16-bit, namespaced loosely by module) --------------- */
typedef enum {
    /* System */
    LOG_CODE_BOOT                       = 0x0001u,
    LOG_CODE_WATCHDOG_RESET_DETECTED    = 0x0002u,
    LOG_CODE_BROWNOUT_RESET_DETECTED    = 0x0003u,
    LOG_CODE_SOFT_RESET                 = 0x0004u,

    /* Comms */
    LOG_CODE_BAD_MAGIC                  = 0x0100u,
    LOG_CODE_BAD_VERSION                = 0x0101u,
    LOG_CODE_BAD_CRC                    = 0x0102u,
    LOG_CODE_BAD_LENGTH                 = 0x0103u,
    LOG_CODE_UART_OVERRUN               = 0x0104u,
    LOG_CODE_UART_FRAMING_ERR           = 0x0105u,
    LOG_CODE_UART_NOISE                 = 0x0106u,
    LOG_CODE_TX_QUEUE_FULL              = 0x0107u,
    LOG_CODE_UNKNOWN_PACKET_TYPE        = 0x0108u,
    LOG_CODE_UNKNOWN_CMD_SUBTYPE        = 0x0109u,
    LOG_CODE_PARAM_ID_UNKNOWN           = 0x010Au,
    LOG_CODE_SEQ_GAP                    = 0x010Bu,
    LOG_CODE_REMOTE_NACK                = 0x010Cu,

    /* Motors */
    LOG_CODE_OVERCURRENT_M1             = 0x0200u,
    LOG_CODE_OVERCURRENT_M2             = 0x0201u,

    /* HX711 */
    LOG_CODE_HX711_TIMEOUT              = 0x0500u,
    LOG_CODE_HX711_TARE_COMPLETE        = 0x0501u,

    /* Proximity */
    LOG_CODE_PROX_TRIGGERED             = 0x0700u,
    LOG_CODE_PROX_CLEARED               = 0x0701u,

    /* E-STOP */
    LOG_CODE_ESTOP_ASSERTED             = 0x0800u,
    LOG_CODE_ESTOP_CLEARED              = 0x0801u,
    LOG_CODE_ESTOP_OVERRIDE             = 0x0802u,

    /* Heartbeat */
    LOG_CODE_HEARTBEAT_LOST             = 0x0900u,
    LOG_CODE_HEARTBEAT_GRACE_EXPIRED    = 0x0901u,
    LOG_CODE_HEARTBEAT_RESTORED         = 0x0902u,

    /* State machine */
    LOG_CODE_MODE_TRANSITION            = 0x0A00u,
    LOG_CODE_FUNCTION_TRANSITION        = 0x0A01u,
    LOG_CODE_ILLEGAL_TRANSITION         = 0x0A02u,

    /* Navigation */
    LOG_CODE_LINE_LOST                  = 0x0B03u,
    LOG_CODE_LINE_T_DETECTED            = 0x0B04u,  /* T bar seen → 180° turn (data = black sensor count) */
    LOG_CODE_LINE_REACQUIRED            = 0x0B05u,  /* line found after the turn (data = swept mrad) */
    LOG_CODE_LINE_TURN_FAILED           = 0x0B06u,  /* turn watchdog expired (data = swept mrad) */

    /* Odometry */
    LOG_CODE_ODOMETRY_RESET             = 0x0C00u,

    /* 0x0E00–0x0E04 retired (were TOF — VL53L0X hardware removed) */

    /* Battery (INA219 bus voltage, pushed by the ESP32 over PKT_BATTERY) */
    LOG_CODE_BATTERY_LOW                = 0x0F00u,  /* 3S below caution (data = mV) */
    LOG_CODE_BATTERY_ESTOP              = 0x0F01u,  /* 3S below estop (data = mV) */
    LOG_CODE_BATTERY_RESTORED           = 0x0F02u,  /* 3S recovered above hysteresis */
    /* 0x0F03 retired (was BATTERY_6S_LOW — 6S monitor removed) */
    LOG_CODE_BATTERY_I2C_FAIL           = 0x0F04u,  /* INA219 read failure (legacy I2C path; data = addr) */
    LOG_CODE_BATTERY_STALE              = 0x0F05u,  /* ESP32 push stream stale → treated absent */

    /* LiDAR (Jetson-segmented LaserScan, pushed over PKT_LIDAR_SEGMENTS) */
    LOG_CODE_LIDAR_TRIGGERED            = 0x1000u,  /* entered a caution/estop band (data = mm) */
    LOG_CODE_LIDAR_CLEARED              = 0x1001u,  /* back to NORMAL */
    LOG_CODE_LIDAR_STALE                = 0x1002u,  /* no segments within LIDAR_STALE_MS → treated clear */

    /* Parameters */
    LOG_CODE_PARAM_APPLIED              = 0x0D07u,
} log_code_t;

#endif /* TYPES_H */
