#include "vision_link.h"

#include <string.h>

static uint16_t vl_crc_update(uint16_t crc, uint8_t byte) {
    uint8_t bit;
    crc ^= (uint16_t)byte << 8;
    for (bit = 0u; bit < 8u; ++bit) {
        if ((crc & 0x8000u) != 0u) {
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void vl_put_u16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)(value >> 8);
}

static void vl_put_u32(uint8_t *out, uint32_t value) {
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8) & 0xFFu);
    out[2] = (uint8_t)((value >> 16) & 0xFFu);
    out[3] = (uint8_t)(value >> 24);
}

static uint16_t vl_get_u16(const uint8_t *data) {
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t vl_get_u32(const uint8_t *data) {
    return (uint32_t)data[0]
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24);
}

uint16_t vl_crc16_ccitt_false(const uint8_t *data, size_t length) {
    size_t index;
    uint16_t crc = 0xFFFFu;
    if (data == NULL && length != 0u) {
        return 0u;
    }
    for (index = 0u; index < length; ++index) {
        crc = vl_crc_update(crc, data[index]);
    }
    return crc;
}

size_t vl_pack_frame(uint8_t type, uint8_t seq, const uint8_t *payload,
                     uint8_t payload_length, uint8_t *out, size_t out_capacity) {
    size_t total = (size_t)payload_length + 8u;
    uint16_t crc;
    if (out == NULL || payload_length > VL_MAX_PAYLOAD || out_capacity < total) {
        return 0u;
    }
    if (payload_length != 0u && payload == NULL) {
        return 0u;
    }
    out[0] = VL_SOF1;
    out[1] = VL_SOF2;
    out[2] = VL_VERSION;
    out[3] = type;
    out[4] = seq;
    out[5] = payload_length;
    if (payload_length != 0u) {
        memcpy(&out[6], payload, payload_length);
    }
    crc = vl_crc16_ccitt_false(&out[2], (size_t)payload_length + 4u);
    vl_put_u16(&out[6u + payload_length], crc);
    return total;
}

size_t vl_pack_observation(uint8_t seq, const vl_observation_t *observation,
                           uint8_t *out, size_t out_capacity) {
    uint8_t payload[16];
    vl_observation_t value;
    if (observation == NULL) {
        return 0u;
    }
    value = *observation;
    value.flags &= 0x0Fu;
    if ((value.flags & VL_FLAG_TARGET_VALID) == 0u) {
        value.center_x = 0;
        value.center_y = 0;
        value.error_x = 0;
        value.error_y = 0;
        value.confidence = 0u;
        value.target_id = 0xFFu;
    }
    if (value.confidence > 100u) {
        value.confidence = 100u;
    }
    vl_put_u32(&payload[0], value.timestamp_ms);
    vl_put_u16(&payload[4], (uint16_t)value.center_x);
    vl_put_u16(&payload[6], (uint16_t)value.center_y);
    vl_put_u16(&payload[8], (uint16_t)value.error_x);
    vl_put_u16(&payload[10], (uint16_t)value.error_y);
    payload[12] = value.confidence;
    payload[13] = value.flags;
    payload[14] = value.target_id;
    payload[15] = value.mode;
    return vl_pack_frame(VL_TYPE_VISION_OBSERVATION, seq, payload, 16u,
                         out, out_capacity);
}

size_t vl_pack_heartbeat(uint8_t seq, const vl_heartbeat_t *heartbeat,
                         uint8_t *out, size_t out_capacity) {
    uint8_t payload[8];
    if (heartbeat == NULL) {
        return 0u;
    }
    vl_put_u32(&payload[0], heartbeat->timestamp_ms);
    vl_put_u16(&payload[4], heartbeat->status_bits);
    vl_put_u16(&payload[6], heartbeat->fps_x10);
    return vl_pack_frame(VL_TYPE_HEARTBEAT, seq, payload, 8u, out, out_capacity);
}

size_t vl_pack_command(uint8_t seq, const vl_command_t *command,
                       uint8_t *out, size_t out_capacity) {
    uint8_t payload[8];
    if (command == NULL) {
        return 0u;
    }
    payload[0] = command->command_id;
    payload[1] = command->mode;
    vl_put_u16(&payload[2], (uint16_t)command->arg0);
    vl_put_u16(&payload[4], (uint16_t)command->arg1);
    vl_put_u16(&payload[6], (uint16_t)command->arg2);
    return vl_pack_frame(VL_TYPE_COMMAND, seq, payload, 8u, out, out_capacity);
}

size_t vl_pack_ack(uint8_t seq, const vl_ack_t *ack,
                   uint8_t *out, size_t out_capacity) {
    uint8_t payload[4];
    if (ack == NULL) {
        return 0u;
    }
    payload[0] = ack->request_type;
    payload[1] = ack->request_seq;
    payload[2] = ack->status;
    payload[3] = ack->detail;
    return vl_pack_frame(VL_TYPE_ACK, seq, payload, 4u, out, out_capacity);
}

int vl_decode_observation(const vl_frame_t *frame, vl_observation_t *out) {
    if (frame == NULL || out == NULL || frame->version != VL_VERSION
        || frame->type != VL_TYPE_VISION_OBSERVATION || frame->length != 16u) {
        return 0;
    }
    out->timestamp_ms = vl_get_u32(&frame->payload[0]);
    out->center_x = (int16_t)vl_get_u16(&frame->payload[4]);
    out->center_y = (int16_t)vl_get_u16(&frame->payload[6]);
    out->error_x = (int16_t)vl_get_u16(&frame->payload[8]);
    out->error_y = (int16_t)vl_get_u16(&frame->payload[10]);
    out->confidence = frame->payload[12];
    out->flags = frame->payload[13];
    out->target_id = frame->payload[14];
    out->mode = frame->payload[15];
    return 1;
}

int vl_decode_heartbeat(const vl_frame_t *frame, vl_heartbeat_t *out) {
    if (frame == NULL || out == NULL || frame->version != VL_VERSION
        || frame->type != VL_TYPE_HEARTBEAT || frame->length != 8u) {
        return 0;
    }
    out->timestamp_ms = vl_get_u32(&frame->payload[0]);
    out->status_bits = vl_get_u16(&frame->payload[4]);
    out->fps_x10 = vl_get_u16(&frame->payload[6]);
    return 1;
}

int vl_decode_command(const vl_frame_t *frame, vl_command_t *out) {
    if (frame == NULL || out == NULL || frame->version != VL_VERSION
        || frame->type != VL_TYPE_COMMAND || frame->length != 8u) {
        return 0;
    }
    out->command_id = frame->payload[0];
    out->mode = frame->payload[1];
    out->arg0 = (int16_t)vl_get_u16(&frame->payload[2]);
    out->arg1 = (int16_t)vl_get_u16(&frame->payload[4]);
    out->arg2 = (int16_t)vl_get_u16(&frame->payload[6]);
    return 1;
}

int vl_decode_ack(const vl_frame_t *frame, vl_ack_t *out) {
    if (frame == NULL || out == NULL || frame->version != VL_VERSION
        || frame->type != VL_TYPE_ACK || frame->length != 4u) {
        return 0;
    }
    out->request_type = frame->payload[0];
    out->request_seq = frame->payload[1];
    out->status = frame->payload[2];
    out->detail = frame->payload[3];
    return 1;
}

void vl_parser_init(vl_parser_t *parser) {
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->state = VL_WAIT_SOF1;
    parser->crc = 0xFFFFu;
}

void vl_parser_timeout(vl_parser_t *parser) {
    vl_parser_init(parser);
}

vl_parse_result_t vl_parser_push_byte(vl_parser_t *parser, uint8_t byte,
                                      vl_frame_t *out_frame) {
    if (parser == NULL || out_frame == NULL) {
        return VL_PARSE_NONE;
    }
    switch (parser->state) {
        case VL_WAIT_SOF1:
            if (byte == VL_SOF1) {
                parser->state = VL_WAIT_SOF2;
            }
            break;
        case VL_WAIT_SOF2:
            if (byte == VL_SOF2) {
                parser->state = VL_READ_VERSION;
                parser->crc = 0xFFFFu;
            } else if (byte != VL_SOF1) {
                parser->state = VL_WAIT_SOF1;
            }
            break;
        case VL_READ_VERSION:
            if (byte != VL_VERSION) {
                vl_parser_init(parser);
                if (byte == VL_SOF1) {
                    parser->state = VL_WAIT_SOF2;
                }
                return VL_PARSE_BAD_VERSION;
            }
            parser->frame.version = byte;
            parser->crc = vl_crc_update(parser->crc, byte);
            parser->state = VL_READ_TYPE;
            break;
        case VL_READ_TYPE:
            parser->frame.type = byte;
            parser->crc = vl_crc_update(parser->crc, byte);
            parser->state = VL_READ_SEQ;
            break;
        case VL_READ_SEQ:
            parser->frame.seq = byte;
            parser->crc = vl_crc_update(parser->crc, byte);
            parser->state = VL_READ_LENGTH;
            break;
        case VL_READ_LENGTH:
            parser->frame.length = byte;
            parser->crc = vl_crc_update(parser->crc, byte);
            parser->payload_index = 0u;
            if (byte > VL_MAX_PAYLOAD) {
                vl_parser_init(parser);
                return VL_PARSE_BAD_LENGTH;
            }
            parser->state = (byte == 0u) ? VL_READ_CRC_LOW : VL_READ_PAYLOAD;
            break;
        case VL_READ_PAYLOAD:
            parser->frame.payload[parser->payload_index++] = byte;
            parser->crc = vl_crc_update(parser->crc, byte);
            if (parser->payload_index == parser->frame.length) {
                parser->state = VL_READ_CRC_LOW;
            }
            break;
        case VL_READ_CRC_LOW:
            parser->received_crc = byte;
            parser->state = VL_READ_CRC_HIGH;
            break;
        case VL_READ_CRC_HIGH:
            parser->received_crc |= (uint16_t)byte << 8;
            if (parser->received_crc == parser->crc) {
                *out_frame = parser->frame;
                vl_parser_init(parser);
                return VL_PARSE_FRAME;
            }
            vl_parser_init(parser);
            return VL_PARSE_BAD_CRC;
        default:
            vl_parser_init(parser);
            break;
    }
    return VL_PARSE_NONE;
}
