#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"

typedef struct {
    uint32_t pulses_ma;       /**< 最近一个100 ms窗口的MA脉冲绝对值。 */
    uint32_t pulses_mb;       /**< 最近一个100 ms窗口的MB脉冲绝对值。 */
    uint32_t pulses_mc;       /**< 最近一个100 ms窗口的MC脉冲绝对值。 */
    uint32_t pulses_md;       /**< 最近一个100 ms窗口的MD脉冲绝对值。 */
    uint8_t low_windows_ma;   /**< MA连续低脉冲窗口数。 */
    uint8_t low_windows_mb;   /**< MB连续低脉冲窗口数。 */
    uint8_t low_windows_mc;   /**< MC连续低脉冲窗口数。 */
    uint8_t low_windows_md;   /**< MD连续低脉冲窗口数。 */
    int16_t command_ma;       /**< 触发检查窗口时MA的PWM命令。 */
    int16_t command_mb;       /**< 触发检查窗口时MB的PWM命令。 */
    int16_t command_mc;       /**< 触发检查窗口时MC的PWM命令。 */
    int16_t command_md;       /**< 触发检查窗口时MD的PWM命令。 */
    uint8_t stalled_mask;     /**< bit0~3依次表示MA~MD触发堵转。 */
} MotorProtection_Status;

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
 * 每次Reset后的前1秒为启动宽限期，不判堵转。宽限结束后，仅当对应车轮
 * 命令绝对值达到配置门限时检查；任一车轮连续多个100 ms窗口脉冲
 * 过少即触发，触发后由主程序进入不可自动恢复的APP_ERROR。
 */
bool MotorProtection_Update(uint32_t now_ms, const Encoder_Delta *delta);

/**
 * @brief 读取最近一个堵转检查窗口的逐轮诊断值。
 *
 * 返回的静态只读结构始终有效，主要用于触发停车时通过HC-05输出原因。
 */
const MotorProtection_Status *MotorProtection_GetStatus(void);

#endif
