#ifndef NEWGREY_H
#define NEWGREY_H

#include <stdint.h>
#include <stdbool.h>

#define GRAY_SENSOR_COUNT 8U

/**
 * @brief 感为I2C八路灰度的一帧数据。
 *
 * value、active_mask都按车体从左到右排列。默认假定S1为最左、S8为最右，
 * 若实机相反，只修改NEWGRAY_REVERSE_ORDER。
 */
typedef struct {
    uint16_t value[GRAY_SENSOR_COUNT]; /**< 0xDD位图展开值：黑=0，白=255。 */
    uint8_t active_mask;               /**< 黑线位图，bit为1表示该路见黑。 */
    uint8_t active_count;              /**< 当前见黑探头数量。 */
    uint8_t raw_mask;                  /**< 传感器0xDD原始返回，1=白、0=黑。 */
} GraySensor_Data;

/**
 * @brief 初始化PC2/PC3软件I2C并ping感为灰度模块。
 * @return 收到0xAA命令对应的0x66响应时返回true。
 *
 * 接线：PC2=SDA、PC3=SCL。传感器板5V上拉跳帽不得安装，应将总线通过
 * 4.7k~10k电阻上拉到3.3V，或者使用双向I2C电平转换器。
 */
bool NewGray_Init(void);

/**
 * @brief 发送0xDD命令并读取八路数字量位图。
 * @param data 输出一帧按左右顺序整理后的灰度数据。
 * @return I2C地址、命令和读取过程全部应答时返回true。
 *
 * 通信失败时不修改为伪造的有效数据，主程序应保持电机停止并继续测试重连。
 */
bool NewGray_Read(GraySensor_Data *data);

/**
 * @brief 获取最近一次I2C通信错误阶段。
 * @return 0=成功，1=写地址NACK，2=命令NACK，3=读地址NACK，4=总线超时。
 */
uint8_t NewGray_LastError(void);

#endif
