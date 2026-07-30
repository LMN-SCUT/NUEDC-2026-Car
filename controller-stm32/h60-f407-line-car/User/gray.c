#include "gray.h"
#include "board.h"
#include "stm32f4xx.h"
#include "timebase.h"

GrayData gray;

/*
 * 感为八路灰度模块软件 I2C 驱动。
 *
 * 本文件逐项移植“循迹成功最终版”的已验证流程，并改写为 STM32F4 SPL：
 *   PC2 = SDA，PC3 = SCL，开漏输出并启用内部上拉；
 *   START -> 写地址 -> 命令 -> REPEATED START -> 读地址 -> 数据 -> STOP；
 *   0xAA 返回 0x66，用于上电握手；
 *   0xDD 返回 1 字节数字量掩码，bit0~bit7 对应 S1~S8；
 *   bit=1 表示白色，bit=0 表示黑色。
 *
 * 时序延时也采用参考工程的固定 200 次空循环，不再使用此前的
 * SCL 等待、9 脉冲恢复和三次自动重试，方便与成功硬件直接对照。
 */
#define SDA_HIGH() GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define SDA_LOW()  GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define SCL_HIGH() GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define SCL_LOW()  GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define SDA_READ() GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)

/* 参考工程约 5 us 的软件 I2C 延时，SCL 约为 25 kHz。 */
static void I2cDelay(void)
{
    volatile int32_t count;
    for (count = 0; count < 200; count++) {
        __NOP();
    }
}

/* SCL 为高时产生 SDA 下降沿。 */
static void I2cStart(void)
{
    SDA_HIGH();
    I2cDelay();
    SCL_HIGH();
    I2cDelay();
    SDA_LOW();
    I2cDelay();
    SCL_LOW();
    I2cDelay();
}

/* SCL 为高时产生 SDA 上升沿。 */
static void I2cStop(void)
{
    SDA_LOW();
    I2cDelay();
    SCL_HIGH();
    I2cDelay();
    SDA_HIGH();
    I2cDelay();
}

/* 写入一个字节；返回 0=收到 ACK，1=收到 NACK。 */
static uint8_t I2cWrite(uint8_t data)
{
    int8_t bit;
    uint8_t nack;

    for (bit = 7; bit >= 0; bit--) {
        if ((data & (uint8_t)(1U << bit)) != 0U) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        I2cDelay();
        SCL_HIGH();
        I2cDelay();
        SCL_LOW();
        I2cDelay();
    }

    SDA_HIGH();
    I2cDelay();
    SCL_HIGH();
    I2cDelay();
    nack = (uint8_t)SDA_READ();
    SCL_LOW();
    I2cDelay();
    return nack;
}

/*
 * 读取一个字节。send_ack=1 表示后面还有数据，发送 ACK；
 * send_ack=0 表示最后一个字节，发送 NACK。
 */
static uint8_t I2cRead(uint8_t send_ack)
{
    uint8_t data = 0U;
    int8_t bit;

    SDA_HIGH();
    for (bit = 7; bit >= 0; bit--) {
        I2cDelay();
        SCL_HIGH();
        I2cDelay();
        if (SDA_READ() != Bit_RESET) {
            data |= (uint8_t)(1U << bit);
        }
        SCL_LOW();
    }

    if (send_ack != 0U) {
        SDA_LOW();
    } else {
        SDA_HIGH();
    }
    I2cDelay();
    SCL_HIGH();
    I2cDelay();
    SCL_LOW();
    I2cDelay();
    SDA_HIGH();
    return data;
}

/*
 * 参考工程的标准命令读取事务。
 * 返回 0=成功；1=写地址 NACK；2=命令 NACK；3=读地址 NACK。
 */
static uint8_t ReadCommand(uint8_t command, uint8_t *buffer, uint8_t length)
{
    uint8_t index;

    I2cStart();
    if (I2cWrite((uint8_t)(GRAY_ADDRESS << 1)) != 0U) {
        I2cStop();
        return 1U;
    }
    if (I2cWrite(command) != 0U) {
        I2cStop();
        return 2U;
    }
    I2cStart();
    if (I2cWrite((uint8_t)((GRAY_ADDRESS << 1) | 1U)) != 0U) {
        I2cStop();
        return 3U;
    }

    for (index = 0U; index < length; index++) {
        buffer[index] =
            I2cRead((uint8_t)(index + 1U < length));
    }
    I2cStop();
    return 0U;
}

void Gray_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t pong = 0U;
    uint8_t dummy = 0U;
    uint8_t tries;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    /* 对齐参考工程 GPIO_SPEED_FREQ_LOW，避免无意义的高速边沿。 */
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &gpio);
    SDA_HIGH();
    SCL_HIGH();

    gray.mask = 0xFFU;
    gray.black_count = 0U;
    gray.line_found = 1U;
    gray.communication_ok = 0U;
    gray.error_stage = 0U;
    gray.error_count = 0U;
    gray.error = 0.0f;

    /* 与参考工程相同：最多 50 次 0xAA/0x66 握手，每次间隔 10 ms。 */
    for (tries = 0U; tries < 50U; tries++) {
        gray.error_stage = ReadCommand(0xAAU, &pong, 1U);
        if ((gray.error_stage == 0U) && (pong == 0x66U)) {
            break;
        }
        DelayMs(10U);
    }
    if (tries >= 50U) {
        gray.error_count++;
        return;
    }

    /* 发送 0xDD 并读取首帧，确认数字量位掩码模式可用。 */
    gray.error_stage = ReadCommand(0xDDU, &dummy, 1U);
    if (gray.error_stage == 0U) {
        gray.communication_ok = 1U;
    } else {
        gray.error_count++;
    }
}

uint8_t Gray_Read(void)
{
    static uint8_t found_samples;
    static uint8_t lost_samples;
    static const int8_t weights[8] = {-8, -6, -3, -1, 1, 3, 6, 8};
    uint8_t mask;
    uint8_t index;
    uint8_t status;
    int16_t weight_sum = 0;

    /*
     * 与参考工程一致：每次发送 0xDD 命令并读取一个掩码字节，
     * 最后一个字节由主机发送 NACK。
     */
    status = ReadCommand(0xDDU, &mask, 1U);
    gray.error_stage = status;
    if (status != 0U) {
        gray.communication_ok = 0U;
        gray.error_count++;
        return 0U;
    }

    gray.communication_ok = 1U;
    gray.mask = mask;
    gray.black_count = 0U;
    for (index = 0U; index < 8U; index++) {
        gray.values[index] =
            ((mask & (uint8_t)(1U << index)) != 0U) ? 255U : 0U;
        if (gray.values[index] < GRAY_THRESHOLD) {
            gray.black_count++;
            weight_sum += weights[index];
        }
    }

    gray.error = (gray.black_count > 0U)
               ? ((float)weight_sum / (float)gray.black_count) : 0.0f;

    /* 与参考工程一致的连续找到/丢失滞后滤波。 */
    if (gray.black_count > 0U) {
        if (found_samples < 255U) {
            found_samples++;
        }
        lost_samples = 0U;
        if (found_samples >= LINE_FOUND_SAMPLES) {
            gray.line_found = 1U;
        }
    } else {
        if (lost_samples < 255U) {
            lost_samples++;
        }
        found_samples = 0U;
        if (lost_samples >= LINE_LOST_SAMPLES) {
            gray.line_found = 0U;
        }
    }
    return 1U;
}
