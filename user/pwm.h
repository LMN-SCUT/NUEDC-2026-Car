#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f10x.h"

void PWM_TIM3_Init(void);
void PWM_Set_Left(uint16_t speed);
void PWM_Set_Right(uint16_t speed);

#endif 