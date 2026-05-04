#include "frame.h"
#include "crc.h"
#include <string.h>

frame_result_t frame_validate(const uint8_t *frame, size_t total_len,
                              frame_header_t *header) {
    if (total_len < 8) return FRAME_BAD_LENGTH;
    if (frame[0] != PROTO_MAGIC0 || frame[1] != PROTO_MAGIC1) return FRAME_BAD_MAGIC;

    uint8_t ver = frame[2];
    uint8_t seq = frame[3];
    uint8_t typ = frame[4];
    uint8_t len = frame[5];

    if (header) { header->ver = ver; header->seq = seq; header->type = typ; header->len = len; }

    if (total_len != (size_t)(8u + len)) return FRAME_BAD_LENGTH;
    if (ver != PROTO_VERSION)            return FRAME_BAD_VERSION;

    uint16_t crc_calc = crc16_compute(&frame[2], 4u + (uint32_t)len);
    uint16_t crc_recv = (uint16_t)frame[6 + len] | ((uint16_t)frame[6 + len + 1] << 8);
    if (crc_calc != crc_recv) return FRAME_BAD_CRC;

    return FRAME_OK;
}

size_t frame_build_nack(uint8_t out[PROTO_MAX_FRAME], uint8_t seq, uint8_t err) {
    out[0] = PROTO_MAGIC0;
    out[1] = PROTO_MAGIC1;
    out[2] = PROTO_VERSION;
    out[3] = 0;            /* relay-originated SEQ; workstation/STM32 don't track this stream */
    out[4] = PKT_NACK;
    out[5] = 2;
    out[6] = seq;
    out[7] = err;
    uint16_t crc = crc16_compute(&out[2], 4u + 2u);
    out[8] = (uint8_t)(crc & 0xFFu);
    out[9] = (uint8_t)(crc >> 8);
    return 10;
}

/* ===========================================================================
 *  Streaming parser
 * =========================================================================== */

void frame_parser_init(frame_parser_t *p,
                       frame_complete_cb_t on_complete,
                       frame_error_cb_t on_error) {
    memset(p, 0, sizeof(*p));
    p->state = frame_parser_t::S_MAGIC0;
    p->on_complete = on_complete;
    p->on_error    = on_error;
}

static inline void parser_reset(frame_parser_t *p) {
    p->state = frame_parser_t::S_MAGIC0;
    p->payload_idx = 0;
    p->buf_idx = 0;
}

static inline void parser_buf_push(frame_parser_t *p, uint8_t b) {
    if (p->buf_idx < PROTO_MAX_FRAME) p->buf[p->buf_idx++] = b;
}

void frame_parser_feed(frame_parser_t *p, uint8_t b) {
    switch (p->state) {
    case frame_parser_t::S_MAGIC0:
        if (b == PROTO_MAGIC0) {
            p->buf_idx = 0;
            parser_buf_push(p, b);
            p->state = frame_parser_t::S_MAGIC1;
        }
        break;

    case frame_parser_t::S_MAGIC1:
        if (b == PROTO_MAGIC1) {
            parser_buf_push(p, b);
            p->state = frame_parser_t::S_VER;
        } else {
            if (p->on_error) p->on_error(FRAME_BAD_MAGIC, 0);
            /* If b itself is MAGIC0, restart at MAGIC1 with this byte. */
            parser_reset(p);
            if (b == PROTO_MAGIC0) {
                parser_buf_push(p, b);
                p->state = frame_parser_t::S_MAGIC1;
            }
        }
        break;

    case frame_parser_t::S_VER:
        p->ver = b;
        parser_buf_push(p, b);
        if (b != PROTO_VERSION) {
            if (p->on_error) p->on_error(FRAME_BAD_VERSION, 0);
            parser_reset(p);
            break;
        }
        p->state = frame_parser_t::S_SEQ;
        break;

    case frame_parser_t::S_SEQ:
        p->seq = b;
        parser_buf_push(p, b);
        p->state = frame_parser_t::S_TYPE;
        break;

    case frame_parser_t::S_TYPE:
        p->type = b;
        parser_buf_push(p, b);
        p->state = frame_parser_t::S_LEN;
        break;

    case frame_parser_t::S_LEN:
        p->len = b;
        parser_buf_push(p, b);
        if (p->len == 0) {
            p->state = frame_parser_t::S_CRC_LO;
        } else {
            p->payload_idx = 0;
            p->state = frame_parser_t::S_PAYLOAD;
        }
        break;

    case frame_parser_t::S_PAYLOAD:
        parser_buf_push(p, b);
        p->payload_idx++;
        if (p->payload_idx >= p->len) p->state = frame_parser_t::S_CRC_LO;
        break;

    case frame_parser_t::S_CRC_LO:
        parser_buf_push(p, b);
        p->crc_received = b;
        p->state = frame_parser_t::S_CRC_HI;
        break;

    case frame_parser_t::S_CRC_HI: {
        parser_buf_push(p, b);
        p->crc_received |= ((uint16_t)b << 8);

        uint16_t crc_calc = crc16_compute(&p->buf[2], 4u + (uint32_t)p->len);
        if (crc_calc == p->crc_received) {
            if (p->on_complete) p->on_complete(p->buf, p->buf_idx);
        } else {
            if (p->on_error) p->on_error(FRAME_BAD_CRC, p->seq);
        }
        parser_reset(p);
        break;
    }
    }
}
