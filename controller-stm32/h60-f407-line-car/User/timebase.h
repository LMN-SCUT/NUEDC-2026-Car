#ifndef __TIMEBASE_H
#define __TIMEBASE_H

#include <stdint.h>

void Timebase_Init(void);
uint32_t Millis(void);
void DelayMs(uint32_t ms);
void DelayUs(uint32_t us);

#endif
