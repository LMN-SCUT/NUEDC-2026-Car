#include "vision_link.h"

#include <stdio.h>
#include <string.h>

static const uint8_t expected[] = {
    0xAA, 0x55, 0x01, 0x01, 0x05, 0x10, 0xE8, 0x03,
    0x00, 0x00, 0x40, 0x01, 0xF0, 0x00, 0x0A, 0x00,
    0xFB, 0xFF, 0x5A, 0x01, 0x00, 0x00, 0xBF, 0xBE
};

int main(void) {
    vl_observation_t observation;
    vl_observation_t decoded;
    vl_parser_t parser;
    vl_frame_t frame;
    uint8_t output[VL_MAX_FRAME_SIZE];
    size_t length;
    size_t index;
    int frame_count = 0;

    memset(&observation, 0, sizeof(observation));
    observation.timestamp_ms = 1000u;
    observation.center_x = 320;
    observation.center_y = 240;
    observation.error_x = 10;
    observation.error_y = -5;
    observation.confidence = 90u;
    observation.flags = VL_FLAG_TARGET_VALID;

    length = vl_pack_observation(5u, &observation, output, sizeof(output));
    if (length != sizeof(expected) || memcmp(output, expected, sizeof(expected)) != 0) {
        fprintf(stderr, "C pack does not match standard vector\n");
        return 1;
    }

    vl_parser_init(&parser);
    for (index = 0u; index < length; ++index) {
        if (vl_parser_push_byte(&parser, output[index], &frame) == VL_PARSE_FRAME) {
            ++frame_count;
        }
    }
    if (frame_count != 1 || !vl_decode_observation(&frame, &decoded)) {
        fprintf(stderr, "C parser did not recover one observation\n");
        return 2;
    }
    if (decoded.timestamp_ms != 1000u || decoded.center_x != 320
        || decoded.center_y != 240 || decoded.error_x != 10
        || decoded.error_y != -5 || decoded.confidence != 90u) {
        fprintf(stderr, "C decoded fields mismatch\n");
        return 3;
    }

    output[10] ^= 0x80u;
    vl_parser_init(&parser);
    for (index = 0u; index < length; ++index) {
        if (vl_parser_push_byte(&parser, output[index], &frame) == VL_PARSE_FRAME) {
            fprintf(stderr, "C parser accepted bad CRC\n");
            return 4;
        }
    }

    puts("c_protocol_tests=PASS vector=PASS crc_rejection=PASS");
    return 0;
}
