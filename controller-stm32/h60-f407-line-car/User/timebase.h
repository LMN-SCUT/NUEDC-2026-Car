#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/**
 * @brief 初始化1 ms系统时基。
 *
 * 使用Cortex-M4 SysTick，每1 ms进入一次SysTick_Handler。
 * 应在使用Timebase_Millis或Timebase_DelayMs之前调用一次。
 */
void Timebase_Init(void);

/**
 * @brief 获取从Timebase_Init开始累计的毫秒数。
 * @return 32位毫秒计数，约49.7天后自然回绕。
 *
 * 用无符号减法比较时间差，可正确处理计数回绕。
 */
uint32_t Timebase_Millis(void);

/**
 * @brief 阻塞等待指定毫秒数。
 * @param milliseconds 等待时间，单位ms。
 *
 * 当前仅用于4051换路后等待模拟输出稳定。该函数等待期间CPU不执行
 * 其他主循环任务，不应用于很长的延时。
 */
void Timebase_DelayMs(uint32_t milliseconds);

#endif
