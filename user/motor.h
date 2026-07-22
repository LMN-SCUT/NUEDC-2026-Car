#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);
void Limit(int *motor_left,int *motor_right);
void Motor_Set_PWM(int motor_l, int motor_r);
#endif
