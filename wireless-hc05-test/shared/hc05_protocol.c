#include "hc05_protocol.h"

uint8_t hc05_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0u;
    uint8_t index;
    uint8_t bit;

    for (index = 0u; index < length; ++index) {
        crc ^= data[index];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0x07u)
                                : (uint8_t)(crc << 1u);
        }
    }
    return crc;
}

void hc05_parser_init(hc05_parser_t *parser)
{
    parser->state = 0u;
    parser->payload_index = 0u;
}

uint8_t hc05_frame_pack(const hc05_frame_t *frame, uint8_t output[HC05_FRAME_SIZE])
{
    output[0] = HC05_FRAME_SOF0;
    output[1] = HC05_FRAME_SOF1;
    output[2] = frame->type;
    output[3] = frame->node_id;
    output[4] = (uint8_t)(frame->sequence & 0xFFu);
    output[5] = (uint8_t)(frame->sequence >> 8u);
    output[6] = hc05_crc8(&output[2], 4u);
    return HC05_FRAME_SIZE;
}

hc05_parse_result_t hc05_parser_push(hc05_parser_t *parser, uint8_t byte,
                                     hc05_frame_t *frame)
{
    if (parser->state == 0u) {
        parser->state = (byte == HC05_FRAME_SOF0) ? 1u : 0u;
        return HC05_PARSE_NONE;
    }
    if (parser->state == 1u) {
        if (byte == HC05_FRAME_SOF1) {
            parser->state = 2u;
            parser->payload_index = 0u;
        } else {
            parser->state = (byte == HC05_FRAME_SOF0) ? 1u : 0u;
        }
        return HC05_PARSE_NONE;
    }

    parser->payload[parser->payload_index++] = byte;
    if (parser->payload_index < 5u) {
        return HC05_PARSE_NONE;
    }

    parser->state = 0u;
    if (hc05_crc8(parser->payload, 4u) != parser->payload[4]) {
        return HC05_PARSE_BAD;
    }
    frame->type = parser->payload[0];
    frame->node_id = parser->payload[1];
    frame->sequence = (uint16_t)parser->payload[2]
                    | ((uint16_t)parser->payload[3] << 8u);
    return HC05_PARSE_FRAME;
}
