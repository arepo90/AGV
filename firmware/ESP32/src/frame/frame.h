#pragma once

#include <stdint.h>
#include <stddef.h>
#include "config.h"

/* =============================================================================
 *  Frame parser (UART side).
 *
 *  The relay never modifies frame contents — it only validates CRC + version
 *  before forwarding. Callbacks below fire when a complete frame is decoded
 *  or when something is wrong.
 *
 *  WebSocket-side validation is a single function call (validate_frame) since
 *  WS messages arrive in one piece.
 * =============================================================================
 */

typedef enum {
    FRAME_OK,
    FRAME_BAD_MAGIC,
    FRAME_BAD_VERSION,
    FRAME_BAD_CRC,
    FRAME_BAD_LENGTH,
    FRAME_INCOMPLETE,
} frame_result_t;

/* Per-frame metadata extractable without trusting CRC, used for NACK responses. */
typedef struct {
    uint8_t  ver;
    uint8_t  seq;
    uint8_t  type;
    uint8_t  len;
} frame_header_t;

/* Validate a complete frame buffer.
 *  - If FRAME_OK, *header is populated.
 *  - If FRAME_BAD_CRC and total >= 6, *header is populated with the suspect SEQ
 *    so the caller can NACK it (best-effort — corrupt frame, take with salt). */
frame_result_t frame_validate(const uint8_t *frame, size_t total_len,
                              frame_header_t *header);

/* Build a NACK frame ready to send. Returns total length. */
size_t frame_build_nack(uint8_t out[PROTO_MAX_FRAME], uint8_t seq, uint8_t err);

/* =========================================================================
 *  Streaming UART parser
 *  ========================================================================= */

typedef void (*frame_complete_cb_t)(const uint8_t *frame, size_t total_len);
typedef void (*frame_error_cb_t)(frame_result_t err, uint8_t suspect_seq);

typedef struct {
    enum {
        S_MAGIC0, S_MAGIC1, S_VER, S_SEQ, S_TYPE, S_LEN,
        S_PAYLOAD, S_CRC_LO, S_CRC_HI,
    } state;
    uint8_t  ver, seq, type, len;
    uint16_t payload_idx;
    uint16_t crc_received;
    uint8_t  buf[PROTO_MAX_FRAME];   /* full frame as it arrives */
    uint16_t buf_idx;

    frame_complete_cb_t on_complete;
    frame_error_cb_t    on_error;
} frame_parser_t;

void frame_parser_init(frame_parser_t *p,
                       frame_complete_cb_t on_complete,
                       frame_error_cb_t on_error);
void frame_parser_feed(frame_parser_t *p, uint8_t b);
