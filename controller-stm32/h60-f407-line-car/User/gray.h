#ifndef __GRAY_H
#define __GRAY_H

#include <stdint.h>

typedef struct {
    uint8_t values[8];
    uint8_t mask;
    uint8_t black_count;
    uint8_t line_found;
    uint8_t communication_ok;
    uint8_t error_stage;
    uint32_t error_count;
    float error;
} GrayData;

extern GrayData gray;

void Gray_Init(void);
uint8_t Gray_Read(void);

#endif
