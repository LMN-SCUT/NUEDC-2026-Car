#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t ma; /**< MA编码器自上次读取后的增量。 */
    int32_t mb; /**< MB编码器自上次读取后的增量。 */
    int32_t mc; /**< MC编码器自上次读取后的增量。 */
    int32_t md; /**< MD编码器自上次读取后的增量。 */
} Encoder_Delta;

/**
 * @brief 初始化H60四路编码器的硬件正交解码。
 *
 * 使用TIM2、TIM3、TIM5、TIM4的Encoder Interface模式，计数A/B两相
 * 全部有效边沿。只完成计数，不假定每圈脉冲数和正方向已经标定。
 */
void Encoder_Init(void);

/**
 * @brief 原子读取四路编码器增量并将四个计数器清零。
 * @param delta 输出结构体指针；传入NULL时不执行任何操作。
 *
 * 应以固定周期调用，返回值可用于测速、堵转判断和后续速度PI。
 * 当前主程序每个控制周期调用一次，尚未换算为实际m/s。
 */
void Encoder_ReadAndReset(Encoder_Delta *delta);

#endif
