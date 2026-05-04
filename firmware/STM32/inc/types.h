#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 * Shared enums / typedefs across the firmware.
 *
 * Wire-visible IDs (mode_id, function_id, packet types, log codes) are STABLE
 * once committed — workstation depends on them. Add to the end, never reorder.
 * =============================================================================
 */

/* ---- Operating mode ------------------------------------------------------- */
typedef enum {
    MODE_SUPERVISED   = 0x00u,
    MODE_UNSUPERVISED = 0x01u,
} agv_mode_t;

/* ---- Navigation function -------------------------------------------------- */
typedef enum {
    FUNC_STANDBY           = 0x00u,
    FUNC_REMOTE_CONTROL    = 0x01u,
    FUNC_LINE_FOLLOW       = 0x02u,
    FUNC_TRAJECTORY_FOLLOW = 0x03u,
} agv_function_t;

/* ---- Caution modifier sources (bitmask) ----------------------------------- */
typedef enum {
    CAUTION_SRC_NONE                = 0,
    CAUTION_SRC_LOAD_OVERWEIGHT     = (1u << 0),
    CAUTION_SRC_LOAD_IMBALANCE      = (1u << 1),
    CAUTION_SRC_UNSUPERVISED_NAV    = (1u << 2),
    CAUTION_SRC_PROXIMITY_NEAR      = (1u << 3),
    CAUTION_SRC_WORKSTATION_FORCED  = (1u << 4),
} caution_source_t;

/* ---- Virtual E-STOP sources (bitmask) ------------------------------------- */
typedef enum {
    ESTOP_SRC_NONE              = 0,
    ESTOP_SRC_PROXIMITY         = (1u << 0),  /* auto-clears */
    ESTOP_SRC_CARGO_OVERLOAD    = (1u << 1),  /* auto-clears */
    ESTOP_SRC_CARGO_IMBALANCE   = (1u << 2),  /* auto-clears */
    ESTOP_SRC_HEARTBEAT_TIMEOUT = (1u << 3),  /* needs explicit clear */
    ESTOP_SRC_WORKSTATION       = (1u << 4),  /* needs explicit clear */
    ESTOP_SRC_OVERCURRENT       = (1u << 5),  /* needs explicit clear */
    ESTOP_SRC_FIRMWARE_FAULT    = (1u << 6),  /* needs explicit clear */
} estop_source_t;

#define ESTOP_AUTOCLEAR_MASK    \
    (ESTOP_SRC_PROXIMITY | ESTOP_SRC_CARGO_OVERLOAD | ESTOP_SRC_CARGO_IMBALANCE)

/* ---- Fault log severity --------------------------------------------------- */
typedef enum {
    LOG_SEV_INFO     = 0u,
    LOG_SEV_WARN     = 1u,
    LOG_SEV_ERROR    = 2u,
    LOG_SEV_CRITICAL = 3u,
} log_severity_t;

/* ---- Fault log source module --------------------------------------------- */
typedef enum {
    LOG_MOD_SYSTEM    = 0u,
    LOG_MOD_COMMS     = 1u,
    LOG_MOD_MOTORS    = 2u,
    LOG_MOD_ENCODERS  = 3u,
    LOG_MOD_ADC       = 4u,
    LOG_MOD_HX711     = 5u,
    LOG_MOD_IMU       = 6u,
    LOG_MOD_PROXIMITY = 7u,
    LOG_MOD_ESTOP     = 8u,
    LOG_MOD_HEARTBEAT = 9u,
    LOG_MOD_STATE     = 10u,
    LOG_MOD_NAV       = 11u,
    LOG_MOD_ODOMETRY  = 12u,
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
    LOG_CODE_FRAGMENT_OUT_OF_ORDER      = 0x010Bu,
    LOG_CODE_FRAGMENT_REASSEMBLY_ERR    = 0x010Cu,

    /* Motors */
    LOG_CODE_OVERCURRENT_M1             = 0x0200u,
    LOG_CODE_OVERCURRENT_M2             = 0x0201u,
    LOG_CODE_PWM_SATURATED              = 0x0202u,

    /* Encoders */
    LOG_CODE_ENCODER_OVERFLOW           = 0x0300u,

    /* HX711 */
    LOG_CODE_HX711_TIMEOUT              = 0x0500u,
    LOG_CODE_HX711_OUT_OF_RANGE         = 0x0501u,

    /* IMU */
    LOG_CODE_IMU_I2C_NACK               = 0x0600u,
    LOG_CODE_IMU_I2C_TIMEOUT            = 0x0601u,
    LOG_CODE_IMU_BUS_RESET              = 0x0602u,
    LOG_CODE_IMU_CALIB_LOST             = 0x0603u,

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
    LOG_CODE_TRAJECTORY_LOADED          = 0x0B00u,
    LOG_CODE_WAYPOINT_REACHED           = 0x0B01u,
    LOG_CODE_TRAJECTORY_COMPLETE        = 0x0B02u,
    LOG_CODE_LINE_LOST                  = 0x0B03u,

    /* Odometry */
    LOG_CODE_ODOMETRY_RESET             = 0x0C00u,

    /* Persistent storage / parameters */
    LOG_CODE_QTR_CAL_BEGIN              = 0x0D00u,
    LOG_CODE_QTR_CAL_END                = 0x0D01u,
    LOG_CODE_QTR_CAL_CANCELED           = 0x0D02u,
    LOG_CODE_QTR_CAL_INSUFFICIENT_RANGE = 0x0D03u,
    LOG_CODE_FLASH_WRITE_FAIL           = 0x0D04u,
    LOG_CODE_FLASH_LOAD_FAIL            = 0x0D05u,
    LOG_CODE_FLASH_LOAD_OK              = 0x0D06u,
    LOG_CODE_PARAM_APPLIED              = 0x0D07u,
} log_code_t;

#endif /* TYPES_H */
