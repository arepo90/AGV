#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "config.h"

/* =============================================================================
 * STM32 ↔ ESP32 packet link.
 *
 * Frame on the wire (8 byte overhead + payload):
 *
 *   ┌──────┬──────┬──────┬─────┬──────┬─────┬─────────┬───────┐
 *   │ MAG0 │ MAG1 │ VER  │ SEQ │ TYPE │ LEN │ PAYLOAD │ CRC16 │
 *   │ 0xAA │ 0x56 │ 0x01 │ ... │ ...  │ ... │ 0..255  │ CCITT │
 *   └──────┴──────┴──────┴─────┴──────┴─────┴─────────┴───────┘
 *
 * CRC covers [VER, SEQ, TYPE, LEN, PAYLOAD...].
 *
 * RX  : DMA circular buffer, polled in comms_poll() from main loop.
 *       No RX interrupt needed — the DMA can't be late.
 * TX  : Per-frame DMA kick. Outbound queue holds up to UART_TX_QUEUE_SLOTS
 *       frames; if full, send is dropped and LOG_CODE_TX_QUEUE_FULL is logged.
 *
 * Main loop pattern:
 *
 *     packet_t p;
 *     while (comms_recv(&p)) { handle_packet(&p); }
 *     ...
 *     comms_send(PKT_TELEMETRY, telemetry_buf, sizeof telemetry_buf);
 * =============================================================================
 */

/* ---- Packet types (first byte of frame TYPE field) ----------------------- */
#define PKT_CMD             0x01u
#define PKT_PARAM_UPDATE    0x02u
#define PKT_TELEMETRY       0x03u
#define PKT_HEARTBEAT       0x04u
#define PKT_ACK             0x05u
#define PKT_NACK            0x06u
#define PKT_FRAG            0x07u
#define PKT_LOG             0x08u   /* STM32 → WS, fault log entry */
#define PKT_RESET           0xFFu

/* ---- CMD sub-type (first byte of CMD payload) ---------------------------- */
#define CMD_SET_FUNCTION            0x01u
#define CMD_SET_MODE                0x02u
#define CMD_VEL_CMD                 0x03u   /* float linear, float angular */
#define CMD_VIRTUAL_ESTOP           0x04u
#define CMD_OVERRIDE_ESTOP_SOURCE   0x05u   /* u8 source bitmask to force-clear */
#define CMD_OVERRIDE_CAUTION        0x06u   /* float scalar (workstation super-user) */
#define CMD_READ_SENSOR             0x07u   /* u8 sensor_id */
#define CMD_LOAD_TRAJECTORY         0x08u   /* fragmented upload — see PKT_FRAG */
#define CMD_START_TARE              0x09u   /* enter weight-setting period */
#define CMD_LOG_DUMP_REQUEST        0x0Au
#define CMD_LOG_CLEAR               0x0Bu
#define CMD_QTR_CALIBRATE           0x0Cu   /* run line-sensor calibration, persist to flash */
#define CMD_RESET_ODOMETRY          0x0Du   /* zero pose at current location */

/* ---- PARAM_UPDATE param IDs ---------------------------------------------- */
#define PARAM_MAX_LINEAR_SPEED          0x01u
#define PARAM_MAX_ANGULAR_SPEED         0x02u
#define PARAM_MAX_LINEAR_ACCEL          0x03u
#define PARAM_MAX_ANGULAR_ACCEL         0x04u
#define PARAM_TELEMETRY_RATE_HZ         0x05u
#define PARAM_HEARTBEAT_TIMEOUT_MS      0x06u
#define PARAM_INNER_KP_LEFT             0x10u
#define PARAM_INNER_KI_LEFT             0x11u
#define PARAM_INNER_KD_LEFT             0x12u
#define PARAM_INNER_KP_RIGHT            0x13u
#define PARAM_INNER_KI_RIGHT            0x14u
#define PARAM_INNER_KD_RIGHT            0x15u
#define PARAM_OUTER_LIN_KP              0x16u
#define PARAM_OUTER_LIN_KI              0x17u
#define PARAM_OUTER_LIN_KD              0x18u
#define PARAM_OUTER_ANG_KP              0x19u
#define PARAM_OUTER_ANG_KI              0x1Au
#define PARAM_OUTER_ANG_KD              0x1Bu
#define PARAM_LINE_KP                   0x20u
#define PARAM_LINE_KI                   0x21u
#define PARAM_LINE_KD                   0x22u
#define PARAM_LINE_CRUISE_MPS           0x23u
#define PARAM_TRAJ_CRUISE_MPS           0x24u
#define PARAM_TRAJ_LOOKAHEAD_M          0x25u
#define PARAM_WEIGHT_CAUTION_KG         0x30u
#define PARAM_WEIGHT_ESTOP_KG           0x31u
#define PARAM_IMBALANCE_CAUTION         0x32u
#define PARAM_IMBALANCE_ESTOP           0x33u
/* HX711 per-corner: the value field is f32 on the wire — for offsets we cast
 * to int32 internally. The corner index encoded in the LOW NIBBLE; e.g.
 * 0x34..0x37 = offsets for corners 0..3, 0x38..0x3B = scales for corners 0..3. */
#define PARAM_HX711_OFFSET_BASE         0x34u   /* +0..+3 */
#define PARAM_HX711_SCALE_BASE          0x38u   /* +0..+3 */

/* ---- NACK error codes ---------------------------------------------------- */
#define NACK_BAD_CRC                0x01u
#define NACK_BAD_LENGTH             0x02u
#define NACK_UNKNOWN_TYPE           0x03u
#define NACK_UNKNOWN_SUBTYPE        0x04u
#define NACK_UNKNOWN_PARAM          0x05u
#define NACK_ILLEGAL_TRANSITION     0x06u
#define NACK_BUSY                   0x07u

/* ---- Decoded packet handed to main loop ---------------------------------- */
typedef struct {
    uint8_t  type;
    uint8_t  seq;
    uint8_t  len;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
} packet_t;

void     comms_init(void);
void     comms_poll(void);
bool     comms_recv(packet_t *out);

/* Encode + queue a frame. Returns false if queue is full (and logs the drop). */
bool     comms_send(uint8_t type, const uint8_t *payload, uint8_t len);

/* Convenience for ACK/NACK. seq is the SEQ of the frame being acknowledged. */
bool     comms_send_ack(uint8_t seq, uint8_t status);
bool     comms_send_nack(uint8_t seq, uint8_t error_code);

#endif /* COMMS_H */
