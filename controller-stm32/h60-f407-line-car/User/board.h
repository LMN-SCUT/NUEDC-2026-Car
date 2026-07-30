#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f4xx.h"

/* 电机 PWM 周期为 2 kHz，比较值范围 0~999。 */
#define MOTOR_PWM_TOP              999U

/* 本题地图参数，仅用于说明和停车超时，不用编码器切换赛道状态。 */
#define TRACK_STRAIGHT_MM          1500U
#define TRACK_ARC_RADIUS_MM        500U
#define RUN_TIMEOUT_MS             20000U
#define FINISH_MIN_TIME_MS         6000U

/* 成功代码的控制参数：逻辑速度单位只用于映射占空比。 */
#define TRACK_BASE_SPEED           80.0f
#define TRACK_MAX_SPEED            200.0f
#define TRACK_MIN_DUTY             180U
#define TRACK_KP                   35.0f

/*
 * 与“循迹成功最终版”实际控制一致：
 * diff=35*error，差速限制为基础速度的两倍，即 +/-160。
 * 因此最强输出为外侧饱和 100%、内侧反转约 40%。
 */
#define TRACK_DIFF_MAX              (TRACK_BASE_SPEED * 2.0f)

/* 灰度模块：PC2=SDA，PC3=SCL，I2C 地址 0x4C。 */
#define GRAY_ADDRESS               0x4CU
#define GRAY_THRESHOLD             128U
#define LINE_FOUND_SAMPLES         5U
#define LINE_LOST_SAMPLES          30U

/* A 点横向启停线：稳定检测到至少 4 路黑线才作为候选。 */
#define FINISH_BLACK_MIN           4U
#define FINISH_STABLE_SAMPLES      6U

void Board_Init(void);

#endif
