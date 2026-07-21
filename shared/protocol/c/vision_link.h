#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL_SOF1 0xAAu
#define VL_SOF2 0x55u
#define VL_VERSION 0x01u
#define VL_MAX_PAYLOAD 64u
#define VL_MAX_FRAME_SIZE (8u + VL_MAX_PAYLOAD)

#define VL_TYPE_VISION_OBSERVATION 0x01u
#define VL_TYPE_HEARTBEAT 0x02u
#define VL_TYPE_COMMAND 0x10u
#define VL_TYPE_ACK 0x90u

#define VL_FLAG_TARGET_VALID (1u << 0)
#define VL_FLAG_RESULT_STABLE (1u << 1)
#define VL_FLAG_VALUE_SATURATED (1u << 2)
#define VL_FLAG_PROCESSING_DEGRADED (1u << 3)

typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t seq;
    uint8_t length;
    uint8_t payload[VL_MAX_PAYLOAD];
} vl_frame_t;

typedef struct {
    uint32_t timestamp_ms;
    int16_t center_x;
    int16_t center_y;
    int16_t error_x;
    int16_t error_y;
    uint8_t confidence;
    uint8_t flags;
    uint8_t target_id;
    uint8_t mode;
} vl_observation_t;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t status_bits;
    uint16_t fps_x10;
} vl_heartbeat_t;

typedef struct {
    uint8_t command_id;
    uint8_t mode;
    int16_t arg0;
    int16_t arg1;
    int16_t arg2;
} vl_command_t;

typedef struct {
    uint8_t request_type;
    uint8_t request_seq;
    uint8_t status;
    uint8_t detail;
} vl_ack_t;

typedef enum {
    VL_PARSE_NONE = 0,
    VL_PARSE_FRAME = 1,
    VL_PARSE_BAD_VERSION = -1,
    VL_PARSE_BAD_LENGTH = -2,
    VL_PARSE_BAD_CRC = -3
} vl_parse_result_t;

typedef enum {
    VL_WAIT_SOF1 = 0,
    VL_WAIT_SOF2,
    VL_READ_VERSION,
    VL_READ_TYPE,
    VL_READ_SEQ,
    VL_READ_LENGTH,
    VL_READ_PAYLOAD,
    VL_READ_CRC_LOW,
    VL_READ_CRC_HIGH
} vl_parse_state_t;

typedef struct {
    vl_parse_state_t state;
    vl_frame_t frame;
    uint8_t payload_index;
    uint16_t crc;
    uint16_t received_crc;
} vl_parser_t;

uint16_t vl_crc16_ccitt_false(const uint8_t *data, size_t length);

size_t vl_pack_frame(uint8_t type, uint8_t seq, const uint8_t *payload,
                     uint8_t payload_length, uint8_t *out, size_t out_capacity);
size_t vl_pack_observation(uint8_t seq, const vl_observation_t *observation,
                           uint8_t *out, size_t out_capacity);
size_t vl_pack_heartbeat(uint8_t seq, const vl_heartbeat_t *heartbeat,
                         uint8_t *out, size_t out_capacity);
size_t vl_pack_command(uint8_t seq, const vl_command_t *command,
                       uint8_t *out, size_t out_capacity);
size_t vl_pack_ack(uint8_t seq, const vl_ack_t *ack,
                   uint8_t *out, size_t out_capacity);

int vl_decode_observation(const vl_frame_t *frame, vl_observation_t *out);
int vl_decode_heartbeat(const vl_frame_t *frame, vl_heartbeat_t *out);
int vl_decode_command(const vl_frame_t *frame, vl_command_t *out);
int vl_decode_ack(const vl_frame_t *frame, vl_ack_t *out);

void vl_parser_init(vl_parser_t *parser);
void vl_parser_timeout(vl_parser_t *parser);
vl_parse_result_t vl_parser_push_byte(vl_parser_t *parser, uint8_t byte,
                                      vl_frame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif
