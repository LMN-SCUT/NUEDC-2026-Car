#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32f10x.h"

void Delay_Init(void);       // 延时模块初始化
void Delay_TickInc(void);    // 时基递增（在SysTick中断中调用）
uint32_t Delay_GetTick(void); // 获取当前时间戳（ms）
void Delay_ms(uint32_t ms);  // ms级延时
void Delay_s(uint32_t s);    // s级延时

#endif
