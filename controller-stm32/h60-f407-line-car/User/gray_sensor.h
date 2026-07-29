#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define GRAY_SENSOR_COUNT 8U

typedef struct {
    /** 从车体最左到最右的八路12位ADC原始值。 */
    uint16_t value[GRAY_SENSOR_COUNT];
    /** 黑线检测位图：bit0对应value[0]，bit7对应value[7]。 */
    uint8_t active_mask;
    /** 当前判定为黑线的探头数量，范围0~8。 */
    uint8_t active_count;
} GraySensor_Data;

/**
 * @brief 初始化外接74HC4051八路灰度模块和ADC1。
 *
 * 当前接线为PA3=EN、PA4/PA5/PA6=地址、PC2=ADC1_IN12。
 * EN初始化为低电平以使能模块。本函数不检查OUT电压是否安全。
 */
void GraySensor_Init(void);

/**
 * @brief 完成一次八通道扫描、平均采样和黑线判定。
 * @param data 用于接收八路原始值、位图和激活数量。
 * @return true表示八路全部采样成功；false表示参数无效或ADC超时。
 *
 * 每次4051换路后等待GRAY_MUX_SETTLE_MS，每路采样
 * GRAY_SAMPLES_PER_CHANNEL次。数组左右顺序和八个阈值必须实机标定。
 */
bool GraySensor_Read(GraySensor_Data *data);

#endif
