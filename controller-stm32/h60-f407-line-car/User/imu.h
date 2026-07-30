#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

typedef struct {
    float gyro_z;
    float yaw;
    uint8_t ready;
    uint32_t error_count;
} ImuData;

extern ImuData imu;

void ImuBus_Init(void);
uint8_t Imu_Init(void);
void Imu_Update(void);

#endif
