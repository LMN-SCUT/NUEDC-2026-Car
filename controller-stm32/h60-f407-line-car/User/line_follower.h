#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>
#include "newgrey.h"

typedef enum {
    LINE_TRACKING = 0,       /**< 正常检测到纵向黑线并执行PD差速。 */
    LINE_WIDE_MARKER,        /**< 至少6路见黑，可能是A点横向线。 */
    LINE_TEMPORARILY_LOST,   /**< 短时丢线，正在保持或定向搜索。 */
    LINE_LOST_STOP,          /**< 从未见线或丢线超时，已经停车。 */
    LINE_ADC_ERROR           /**< 输入参数/采样异常，已经停车。 */
} LineFollower_Status;

/**
 * @brief 清零循迹历史误差、丢线计数和“曾经见线”标志。
 *
 * 上电初始化以及每次重新开始任务前调用。
 */
void LineFollower_Init(void);

/**
 * @brief 根据一帧八路灰度数据计算PD修正并输出左右电机命令。
 * @param gray GraySensor_Read得到的数据，不可为NULL。
 * @return 当前循迹状态，主程序据此决定继续、完成或进入故障。
 *
 * 输入直接采用newgrey.c根据0xDD位图生成的黑线active_mask，不使用ADC阈值。
 * 普通黑线使用加权平均位置误差；偏移增大时提高增益并降低基础速度；
 * 启动前500 ms使用30%基础速度助推；6路以上见黑作为宽黑标志；
 * 持续丢线最终停车。
 */
LineFollower_Status LineFollower_Update(const GraySensor_Data *gray);

/**
 * @brief 获取最近一次有效计算的位置误差。
 * @return 约-7~+7；负值表示黑线在车体左侧，正值表示在右侧。
 */
float LineFollower_Error(void);

#endif
