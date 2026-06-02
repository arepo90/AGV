#include "frame.h"
#include "crc.h"
#include <string.h>

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
