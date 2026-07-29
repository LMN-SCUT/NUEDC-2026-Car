#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/**
 * @brief 初始化H60板载四路AT8236电机PWM。
 *
 * 按H60板背面接口布局与实车方向验证：MB/MD为左侧，MA/MC为右侧。
 * 初始化完成后所有PWM为0；若实际排线交叉，仍需按实车位置重新核对。
 */
void Motor_Init(void);

/**
 * @brief 设置左右两侧电机的开环速度百分比。
 * @param left_percent 左侧MB、MD速度，范围-100~100。
 * @param right_percent 右侧MA、MC速度，范围-100~100。
 *
 * 正负号表示两个转动方向，绝对值表示PWM百分比。实际“正数是否前进”
 * 由四个MOTOR_*_INVERT配置决定，必须架空逐轮验证。
 *
 * 当APP_ENABLE_MOTORS=0时，本接口会忽略速度命令并保持停车。
 */
void Motor_SetSidePercent(int16_t left_percent, int16_t right_percent);

/**
 * @brief 分别设置MA、MB、MC、MD四个电机的PWM百分比。
 *
 * 编码器PI使用本接口分别补偿四个电机；灰度直接PWM对照模式则统一调用
 * Motor_SetSidePercent。两种模式由编译开关互斥，不能在同一周期争抢PWM。
 */
void Motor_SetWheelPercent(int16_t ma_percent, int16_t mb_percent,
                           int16_t mc_percent, int16_t md_percent);

/**
 * @brief 关闭所有电机PWM，使AT8236的两个输入均为低电平。
 *
 * 用于上电等待、普通停车和故障停车。此方式允许电机自由减速，
 * 与Motor_Brake的主动短刹车不同。
 */
void Motor_Stop(void);

/**
 * @brief 对四个电机执行AT8236短刹车。
 *
 * 两个输入同时输出高电平以实现制动。仅在APP_ENABLE_MOTORS=1时生效；
 * 当前用于确认终点后的初步停车，制动距离仍需实车标定。
 */
void Motor_Brake(void);

/**
 * @brief 获取最近一次实际允许输出的左侧速度命令。
 * @return 左侧MB/MD中绝对值较大的速度命令。停车后返回0。
 *
 * 主要供堵转保护判断当前是否确实要求左侧车轮转动。
 */
int16_t Motor_LeftCommand(void);

/**
 * @brief 获取最近一次实际允许输出的右侧速度命令。
 * @return 右侧MA/MC中绝对值较大的速度命令。停车后返回0。
 */
int16_t Motor_RightCommand(void);

/** @brief 获取MA当前PWM百分比命令，供逐轮堵转诊断。 */
int16_t Motor_MACommand(void);
/** @brief 获取MB当前PWM百分比命令，供逐轮堵转诊断。 */
int16_t Motor_MBCommand(void);
/** @brief 获取MC当前PWM百分比命令，供逐轮堵转诊断。 */
int16_t Motor_MCCommand(void);
/** @brief 获取MD当前PWM百分比命令，供逐轮堵转诊断。 */
int16_t Motor_MDCommand(void);

#endif
