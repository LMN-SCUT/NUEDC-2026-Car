#ifndef HC05_PROTOCOL_H
#define HC05_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define HC05_FRAME_SOF0 0xA5u
#define HC05_FRAME_SOF1 0x5Au
#define HC05_FRAME_SIZE 7u

typedef enum {
    HC05_FRAME_PING = 0x10u,
    HC05_FRAME_PONG = 0x11u
} hc05_frame_type_t;

typedef struct {
    uint8_t type;
    uint8_t node_id;
    uint16_t sequence;
} hc05_frame_t;

typedef struct {
    uint8_t state;
    uint8_t payload[5];
    uint8_t payload_index;
} hc05_parser_t;

typedef enum {
    HC05_PARSE_NONE = 0,
    HC05_PARSE_FRAME = 1,
    HC05_PARSE_BAD = -1
} hc05_parse_result_t;

uint8_t hc05_crc8(const uint8_t *data, uint8_t length);
void hc05_parser_init(hc05_parser_t *parser);
uint8_t hc05_frame_pack(const hc05_frame_t *frame, uint8_t output[HC05_FRAME_SIZE]);
hc05_parse_result_t hc05_parser_push(hc05_parser_t *parser, uint8_t byte,
                                     hc05_frame_t *frame);

#endif
