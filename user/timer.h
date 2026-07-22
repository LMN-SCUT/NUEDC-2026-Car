#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"

extern volatile uint8_t mpu_flag;
extern volatile uint8_t oled_flag;
void TIM2_Init(void);

#endif
