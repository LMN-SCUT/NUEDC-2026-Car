#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float err;
    float last_err;
    float integral;
    float max_out;
    float max_i;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_i);
float PID_Calc(PID_TypeDef *pid, float target, float current);

#endif
