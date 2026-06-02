#pragma once

#include <stdint.h>
#include <stddef.h>
#include "config.h"

/* =============================================================================
 *  Streaming frame parser. The ESP32 no longer validates or NACKs — that lives
 *  on the Jetson side now. This parser is kept solely to identify complete
 *  PKT_TLM_CORE frames as they pass through, so the local stack-light and
 *  onboard LED can mirror firmware state without reaching back to the Jetson.
 *
 *  The on_complete callback fires for every successfully-parsed frame
 *  (caller filters by TYPE). on_error is best-effort and informational.
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
    uint8_t  buf[PROTO_MAX_FRAME];
    uint16_t buf_idx;

    frame_complete_cb_t on_complete;
    frame_error_cb_t    on_error;
} frame_parser_t;

void frame_parser_init(frame_parser_t *p,
                       frame_complete_cb_t on_complete,
                       frame_error_cb_t on_error);
void frame_parser_feed(frame_parser_t *p, uint8_t b);
