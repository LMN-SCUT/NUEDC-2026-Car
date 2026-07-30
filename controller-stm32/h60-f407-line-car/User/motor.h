#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetSides(float left_speed, float right_speed);
void Motor_StopAll(void);
uint16_t Motor_LeftDuty(void);
uint16_t Motor_RightDuty(void);

#endif
