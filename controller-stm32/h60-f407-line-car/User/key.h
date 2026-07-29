#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/**
 * @brief 初始化H60板载PE1用户按键。
 *
 * PE1配置为带内部上拉的输入，按下接地为低电平。调用本接口前必须先初始化
 * 1 ms系统时基，因为按键消抖使用Timebase_Millis。
 */
void Key_Init(void);

/**
 * @brief 非阻塞检测一次“启动键按下”事件。
 * @return 稳定按下超过30 ms时返回1，其余情况返回0。
 *
 * 一次按住只产生一个事件；必须先松开并稳定消抖，下一次按下才会再次触发。
 * 上电时如果按键已经被按住，不会直接触发，需松开后重新按下。
 */
uint8_t Key_StartPressedEvent(void);

/**
 * @brief 直接读取PE1按键当前状态，供硬件诊断使用。
 * @return 按下为1，松开为0；该接口不做消抖，也不消耗按键事件。
 */
uint8_t Key_IsPressed(void);

#endif
