#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_i)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_i = max_i;
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
}

float PID_Calc(PID_TypeDef *pid, float target, float current) 
{
    pid->err = target - current;
    pid->integral += pid->err;
    if (pid->integral > pid->max_i) pid->integral = pid->max_i;
    if (pid->integral < -pid->max_i) pid->integral = -pid->max_i;
    
    float out = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * (pid->err - pid->last_err);
    if (out > pid->max_out) out = pid->max_out;
    if (out < -pid->max_out) out = -pid->max_out;
    
    pid->last_err = pid->err;
    return out;
}
