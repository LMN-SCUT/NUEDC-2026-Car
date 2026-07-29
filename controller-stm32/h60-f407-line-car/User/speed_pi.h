#ifndef SPEED_PI_H
#define SPEED_PI_H

#include <stdint.h>
#include "encoder.h"

typedef struct {
    int32_t target_ma;
    int32_t target_mb;
    int32_t target_mc;
    int32_t target_md;
    int16_t pwm_ma;
    int16_t pwm_mb;
    int16_t pwm_mc;
    int16_t pwm_md;
} SpeedPI_Status;

/**
 * @brief 初始化并清空四路编码器速度PI。
 *
 * 速度单位统一为“每个20 ms控制周期的编码器计数”，不是rpm。
 */
void SpeedPI_Init(void);

/**
 * @brief 清空积分并停止四个电机。
 *
 * 每次开始新测试、停车或故障后都应调用，避免保留上一次积分。
 */
void SpeedPI_Reset(void);

/**
 * @brief 设置四个车轮的目标编码器增量。
 *
 * 正数表示整车前进方向，负数表示后退，0表示该轮停车。
 */
void SpeedPI_SetTargets(int32_t ma, int32_t mb, int32_t mc, int32_t md);

/**
 * @brief 把灰度外环的左右速度百分比换算为四轮目标编码器计数。
 *
 * left_percent用于MB/MD，right_percent用于MA/MC。百分比在这里表示
 * 相对速度目标而不是PWM，占空比仍由四路PI根据反馈分别计算。
 */
void SpeedPI_SetSidePercentTargets(int16_t left_percent,
                                   int16_t right_percent);

/**
 * @brief 根据最近20 ms的编码器增量执行一次四轮PI并更新PWM。
 * @param measured Encoder_ReadAndReset读取到的四路增量。
 */
void SpeedPI_Update(const Encoder_Delta *measured);

/**
 * @brief 读取当前目标计数和四路PWM，供串口诊断。
 */
const SpeedPI_Status *SpeedPI_GetStatus(void);

#endif
