#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * OpenCTR H60 V3.6 / STM32F407VET6 初步引脚分配。
 *
 * 板载固定资源：
 *   MA: PE9/TIM1_CH1,  PE11/TIM1_CH2
 *   MB: PE13/TIM1_CH3, PE14/TIM1_CH4
 *   MC: PE5/TIM9_CH1,  PE6/TIM9_CH2
 *   MD: PB14/TIM12_CH1, PB15/TIM12_CH2
 *   编码器 MA/MB/MC/MD: TIM2/TIM3/TIM5/TIM4
 *
 * 外接 74HC4051 八路灰度：
 *   EN=PA3, AD0=PA4, AD1=PA5, AD2=PA6, OUT=PC2/ADC1_IN12
 */

/*
 * 当前启用H题地图循迹正式测试分支，APP_MOTOR_TEST_MODE=0。
 * 如需重新做四轮架空/编码器PI隔离测试，再临时改为1。
 */
#define APP_ENABLE_MOTORS             1U
#define APP_MOTOR_TEST_MODE           0U
/*
 * H题循迹控制模式开关：
 * 0=灰度PD目标直接作为左右PWM，编码器仅测速/遥测/堵转保护；
 * 1=灰度PD给速度目标，再由四路编码器PI计算PWM。
 * 当前置0用于判断四轮速度闭环是否限制了弯道转向。
 */
#define APP_ENCODER_SPEED_PI_ENABLE   0U
#define APP_MOTOR_TEST_PERCENT        40
#define APP_MOTOR_TEST_MS             10000U
#define APP_MOTOR_TEST_STALL_CHECK    0U
/*
 * 编码器自检门限：每个1秒报告窗口内，绝对计数达到100即认为该路有脉冲。
 * 该门限只用于判断编码器“有/无信号”，不用于判断四个车轮转速是否一致。
 */
#define ENCODER_TEST_MIN_COUNTS_1S    100

/*
 * 四轮编码器PI初始参数，速度单位是每20 ms的编码器计数。
 * 70计数/20 ms约等于3500计数/秒，低于当前40%落地稳定速度，
 * 适合先进行闭环验证。参数尚未经过实车阶跃响应整定。
 */
#define ENCODER_PI_TEST_ENABLE        1U
#define ENCODER_PI_TEST_TARGET        70
#define ENCODER_PI_TEST_REPORT_MS     200U
#define SPEED_PI_KP                   0.12f
#define SPEED_PI_KI                   0.015f
#define SPEED_PI_FEEDFORWARD_PERCENT_PER_COUNT 0.48f
#define SPEED_PI_COUNTS_PER_PERCENT  2.0f
#define SPEED_PI_INTEGRAL_LIMIT_PERCENT 20.0f
#define SPEED_PI_PWM_LIMIT_PERCENT    65.0f

#define APP_CONTROL_PERIOD_MS         20U
#define APP_TIMED_RUN_MS              30000U
#define APP_FINISH_MIN_MS             5000U
#define APP_START_MARKER_CLEAR_FRAMES 5U
#define APP_FINISH_MARKER_FRAMES      3U
#define APP_TRACK_REPORT_MS           500U
#define APP_LOST_CONFIRM_FRAMES       3U
#define APP_LOST_STOP_FRAMES          50U

/* 初始参数，只用于低速首轮调试。 */
#define LINE_BASE_SPEED_PERCENT       24
#define LINE_MAX_SPEED_PERCENT        35
#define LINE_MIN_FORWARD_PERCENT      10
#define LINE_START_BOOST_PERCENT      30
#define LINE_START_BOOST_MS           500U
#define LINE_SEARCH_OUTER_PERCENT     35
#define LINE_SEARCH_INNER_PERCENT     10
#define LINE_MEDIUM_SPEED_PERCENT     22
#define LINE_LARGE_SPEED_PERCENT      20
#define LINE_WIDE_SPEED_PERCENT       20
#define LINE_KP                       2.5f
#define LINE_KD                       4.0f
#define LINE_MEDIUM_ERROR_LIMIT       3.0f
#define LINE_MEDIUM_GAIN_SCALE        1.2f
#define LINE_LARGE_GAIN_SCALE         1.5f
/*
 * A点启停横线实测常见B=0x3C或0x33，即4路同时见黑；
 * 普通纵向1.8 cm黑线通常只有1~2路见黑。
 */
#define LINE_WIDE_ACTIVE_MIN          4U

/*
 * 当前实车已经确认左侧为MB/MD、右侧为MA/MC，正常配置保持为0。
 * 该宏只保留作方向诊断，不应用它掩盖轮组映射错误。
 */
#define LINE_SWAP_SIDE_COMMANDS       0U

/*
 * 感为I2C八路灰度测试配置。
 * 默认不安装AD0/AD1地址跳帽时7位地址为0x4C；0xDD命令返回数字位图。
 * bit0=S1、bit7=S8，返回位1=白、0=黑。
 */
#define NEWGRAY_I2C_ADDRESS           0x4CU
#define NEWGRAY_REVERSE_ORDER         0U
#define NEWGRAY_I2C_DELAY_US          5U
#define NEWGRAY_PING_RETRIES          50U

/*
 * HC-05使用USART3：PB10=TX、PB11=RX，当前测试参数115200-8-N-1。
 * 灰度测试数据只在电机安全模式下周期发送，避免正式循迹被低速串口阻塞。
 */
#define HC05_BAUD_RATE                115200U
#define HC05_TEST_STREAM_ENABLE       1U
#define HC05_TEST_STREAM_PERIOD_MS    1000U

/*
 * 实车接线位置：MA=右后、MB=左后、MC=右前、MD=左前。
 * 因此左侧是MB/MD，右侧是MA/MC；架空逐轮验证后只修改对应反相宏。
 */
#define MOTOR_MA_INVERT               1U
#define MOTOR_MB_INVERT               0U
#define MOTOR_MC_INVERT               1U
#define MOTOR_MD_INVERT               0U

#define MOTOR_PWM_FREQUENCY_HZ        2000U

/*
 * 编码器正方向实测：整车前进时MA/MC为正，MB/MD为负。
 * 将MB、MD反相后，统一约定整车前进时四路Encoder_Delta均为正。
 */
#define ENCODER_MA_INVERT             0U
#define ENCODER_MB_INVERT             1U
#define ENCODER_MC_INVERT             0U
#define ENCODER_MD_INVERT             1U

/*
 * 编码器堵转保护初始门限。
 * 对应侧命令绝对值达到15%后，每100 ms检查一次；任一车轮少于2个脉冲，
 * 且连续5个检查窗口（500 ms）均满足条件，判定为堵转。
 * 每圈脉冲数尚未实测，正式运行前必须根据最低正常转速重新确认。
 */
#define STALL_PROTECTION_ENABLE       1U
#define STALL_STARTUP_GRACE_MS        1000U
#define STALL_COMMAND_MIN_PERCENT     15
#define STALL_CHECK_PERIOD_MS         100U
#define STALL_MIN_PULSES_PER_WINDOW   2U
#define STALL_CONFIRM_WINDOWS         5U

#endif
