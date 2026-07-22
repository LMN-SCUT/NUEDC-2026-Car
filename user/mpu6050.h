#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

void MPU6050_Init(void);
uint8_t MPU6050_DMP_Get_Yaw(float *yaw); // 

#endif
