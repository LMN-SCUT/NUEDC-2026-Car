#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"

/**
 * @brief 初始化堵转保护内部计数。
 * @param now_ms 当前系统毫秒数。
 *
 * 电机、编码器和时基初始化完成后调用一次。
 */
void MotorProtection_Init(uint32_t now_ms);

/**
 * @brief 清除脉冲累计和连续堵转计数。
 * @param now_ms 当前系统毫秒数。
 *
 * 每次新任务启动前调用，避免把等待阶段的旧计数带入运行阶段。
 */
void MotorProtection_Reset(uint32_t now_ms);

/**
 * @brief 累加本控制周期编码器增量并检查四轮堵转。
 * @param now_ms 当前系统毫秒数。
 * @param delta Encoder_ReadAndReset得到的本周期四路增量。
 * @return true表示已经确认堵转并关闭全部电机；false表示未确认堵转。
 *
 * 每次Reset后的前1秒为启动宽限期，不判堵转。宽限结束后，仅当对应侧
 * 电机命令绝对值达到配置门限时检查；任一车轮连续多个100 ms窗口脉冲
 * 过少即触发，触发后由主程序进入不可自动恢复的APP_ERROR。
 */
bool MotorProtection_Update(uint32_t now_ms, const Encoder_Delta *delta);

#endif
