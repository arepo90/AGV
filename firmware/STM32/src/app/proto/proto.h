#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* =============================================================================
 *  Binary packet protocol over hal/uart.  (app tier)
 *
 *  Frame (8 B overhead + payload):
 *    MAG0 MAG1 VER SEQ TYPE LEN  PAYLOAD[0..255]  CRC16(lo,hi)
 *  CRC-16/CCITT covers [VER, SEQ, TYPE, LEN, PAYLOAD...].
 *
 *  This module merges the old comms+crc+cmd+params: it frames/deframes, parses
 *  the RX byte stream, dispatches commands and parameter updates to the owning
 *  modules, and ACK/NACKs. Telemetry is built in telemetry.c and sent via
 *  proto_send(). RX is drained and dispatched inline (single main-loop context).
 * =============================================================================
 */

/* ---- Packet types --------------------------------------------------------- */
#define PKT_CMD            0x01u
#define PKT_PARAM_UPDATE   0x02u
#define PKT_TLM_CORE       0x03u   /* STM32→WS: operational state + pose (fast) */
#define PKT_HEARTBEAT      0x04u
#define PKT_LIDAR_SEGMENTS 0x07u   /* WS→STM32: Jetson-segmented LaserScan distances (u16 mm[]) */
#define PKT_ACK            0x05u
#define PKT_NACK           0x06u
#define PKT_LOG            0x08u   /* STM32→WS: one fault-log entry */
#define PKT_TLM_DRIVE      0x09u   /* STM32→WS: per-wheel control internals */
#define PKT_TLM_SENSORS    0x0Au   /* STM32→WS: load cells + battery + LiDAR tail */
#define PKT_TLM_QTR        0x0Bu   /* STM32→WS: QTR raw + line position */
#define PKT_BATTERY        0x0Cu   /* ESP32→STM32: pushed INA219 3S bus voltage (u16 mV LE) */
#define PKT_RESET          0xFFu

/* ---- CMD sub-types (first payload byte) ----------------------------------- */
#define CMD_SET_FUNCTION           0x01u
#define CMD_SET_MODE               0x02u
#define CMD_VEL_CMD                0x03u   /* f32 linear, f32 angular */
#define CMD_VIRTUAL_ESTOP          0x04u
#define CMD_OVERRIDE_ESTOP_SOURCE  0x05u   /* u8 mask to force-clear */
#define CMD_OVERRIDE_CAUTION       0x06u   /* f32 scalar */
#define CMD_READ_SENSOR            0x07u   /* not implemented */
#define CMD_START_TARE             0x09u
#define CMD_LOG_DUMP_REQUEST       0x0Au
#define CMD_LOG_CLEAR              0x0Bu
/* 0x0C retired (was QTR_CALIBRATE — QTR is now self-calibrating) */
#define CMD_RESET_ODOMETRY         0x0Du
#define CMD_LOAD_RAMP_CURVE        0x0Eu   /* op 0=begin 1=add(s,f) 2=commit 3=cancel */

/* ---- PARAM_UPDATE ids (u8 id + f32 value tuples) -------------------------- */
#define PARAM_MAX_LINEAR_SPEED     0x01u   /* recognised, not yet plumbed */
#define PARAM_MAX_ANGULAR_SPEED    0x02u   /* recognised, not yet plumbed */
#define PARAM_MAX_LINEAR_ACCEL     0x03u
#define PARAM_MAX_ANGULAR_ACCEL    0x04u
#define PARAM_TELEMETRY_RATE_HZ    0x05u   /* recognised, not yet plumbed */
#define PARAM_HEARTBEAT_TIMEOUT_MS 0x06u   /* recognised, not yet plumbed */
/* Per-wheel velocity PI + feedforward. */
#define PARAM_LEFT_KP              0x10u
#define PARAM_LEFT_KI              0x11u
#define PARAM_LEFT_KFF             0x12u
#define PARAM_RIGHT_KP             0x13u
#define PARAM_RIGHT_KI             0x14u
#define PARAM_RIGHT_KFF            0x15u
/* Line-follow PID + navigator tunables. */
#define PARAM_LINE_KP              0x20u
#define PARAM_LINE_KI              0x21u
#define PARAM_LINE_KD              0x22u
#define PARAM_LINE_CRUISE_MPS      0x23u
#define PARAM_QTR_LINE_LOST_THRESH 0x26u
#define PARAM_LINE_T_BLACK         0x27u   /* T-bar "black" ADC threshold (counts) */
/* Cargo thresholds + HX711 per-corner cal (corner = low nibble). */
#define PARAM_WEIGHT_CAUTION_KG    0x30u
#define PARAM_WEIGHT_ESTOP_KG      0x31u
#define PARAM_IMBALANCE_CAUTION    0x32u
#define PARAM_IMBALANCE_ESTOP      0x33u
#define PARAM_HX711_OFFSET_BASE    0x34u   /* +0..+3 */
#define PARAM_HX711_SCALE_BASE     0x38u   /* +0..+3 */
/* Motion profile (ramp). */
#define PARAM_RAMP_SHAPE           0x40u
#define PARAM_RAMP_JERK_LIN        0x41u
#define PARAM_RAMP_JERK_ANG        0x42u
#define PARAM_RAMP_TAU_LIN         0x43u
#define PARAM_RAMP_TAU_ANG         0x44u
/* Indicator lights (ESP32 rings). All reported in TLM_CORE for the ESP32 tap. */
#define PARAM_LED_MODE             0x50u   /* animation: 0 = pulse, 1 = snake */
#define PARAM_LED_BASE             0x51u   /* indicator ring base: 0 = off, 1 = white */
#define PARAM_LED_INDICATOR_MODE   0x52u   /* indicator spread: 0 = fixed, 1 = responsive */
/* 0x60–0x62 retired (were TOF distance bands — VL53L0X hardware removed). */
/* 3S low-voltage thresholds (mV). */
#define PARAM_BATT_3S_CAUTION_MV   0x63u
#define PARAM_BATT_3S_ESTOP_MV     0x64u
/* LiDAR distance bands (mm). */
#define PARAM_LIDAR_CAUTION_MM     0x65u
#define PARAM_LIDAR_CRITICAL_MM    0x66u
#define PARAM_LIDAR_ESTOP_MM       0x67u

/* ---- NACK error codes ----------------------------------------------------- */
#define NACK_BAD_CRC               0x01u
#define NACK_BAD_LENGTH            0x02u
#define NACK_UNKNOWN_TYPE          0x03u
#define NACK_UNKNOWN_SUBTYPE       0x04u
#define NACK_UNKNOWN_PARAM         0x05u
#define NACK_ILLEGAL_TRANSITION    0x06u
#define NACK_BUSY                  0x07u

void proto_init(void);   /* brings up uart + parser */
void proto_poll(void);   /* drain RX, parse, dispatch, surface UART errors */

/* Encode + queue a frame. False if the TX queue is full. */
bool proto_send(uint8_t type, const uint8_t *payload, uint8_t len);

#endif /* PROTO_H */
