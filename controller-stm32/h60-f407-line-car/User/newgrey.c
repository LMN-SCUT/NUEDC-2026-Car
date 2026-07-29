#include "newgrey.h"
#include "board_config.h"
#include "timebase.h"
#include "stm32f4xx.h"

#define NEWGRAY_COMMAND_PING          0xAAU
#define NEWGRAY_PING_RESPONSE         0x66U
#define NEWGRAY_COMMAND_DIGITAL       0xDDU
#define NEWGRAY_SDA_PIN               2U
#define NEWGRAY_SCL_PIN               3U

static uint8_t last_error;

/* 开漏输出写1表示释放总线，由外部3.3V上拉电阻拉高。 */
static void sda_release(void) { GPIOC->BSRRL = (uint16_t)(1U << NEWGRAY_SDA_PIN); }
static void sda_low(void)     { GPIOC->BSRRH = (uint16_t)(1U << NEWGRAY_SDA_PIN); }
static void scl_release(void) { GPIOC->BSRRL = (uint16_t)(1U << NEWGRAY_SCL_PIN); }
static void scl_low(void)     { GPIOC->BSRRH = (uint16_t)(1U << NEWGRAY_SCL_PIN); }
static uint8_t sda_read(void)
{
    return ((GPIOC->IDR & (1UL << NEWGRAY_SDA_PIN)) != 0U) ? 1U : 0U;
}
static uint8_t scl_read(void)
{
    return ((GPIOC->IDR & (1UL << NEWGRAY_SCL_PIN)) != 0U) ? 1U : 0U;
}

/*
 * 软件I2C半周期延时。I2C允许较低时钟，测试阶段优先保证建立时间。
 * 循环次数按SystemCoreClock缩放，不依赖工程以后是否切换到168MHz。
 */
static void i2c_delay(void)
{
    volatile uint32_t cycles =
        (SystemCoreClock / 1000000U) * NEWGRAY_I2C_DELAY_US;
    if (cycles < 8U) {
        cycles = 8U;
    }
    while (cycles-- > 0U) {
        __NOP();
    }
}

/* 释放SCL并等待实际变高，兼容时钟延展，也能检测总线被永久拉低。 */
static bool scl_high(void)
{
    uint32_t timeout = SystemCoreClock / 1000U;
    scl_release();
    while (scl_read() == 0U) {
        if (timeout-- == 0U) {
            last_error = 4U;
            return false;
        }
    }
    return true;
}

/* START：SCL为高时SDA产生下降沿。 */
static bool i2c_start(void)
{
    sda_release();
    if (!scl_high()) {
        return false;
    }
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    return true;
}

/* STOP：SCL为高时SDA产生上升沿。 */
static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    (void)scl_high();
    i2c_delay();
    sda_release();
    i2c_delay();
}

/* 发送1字节并读取第9位ACK；返回true表示从设备拉低应答。 */
static bool i2c_write_byte(uint8_t value)
{
    int32_t bit;

    for (bit = 7; bit >= 0; bit--) {
        if ((value & (1U << bit)) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        if (!scl_high()) {
            return false;
        }
        i2c_delay();
        scl_low();
    }

    sda_release();
    i2c_delay();
    if (!scl_high()) {
        return false;
    }
    i2c_delay();
    bit = (int32_t)sda_read();
    scl_low();
    return bit == 0;
}

/* 读取1字节；最后一字节send_ack=false，主机发送NACK结束读取。 */
static bool i2c_read_byte(uint8_t *value, bool send_ack)
{
    uint8_t result = 0U;
    int32_t bit;

    if (value == 0) {
        return false;
    }

    sda_release();
    for (bit = 7; bit >= 0; bit--) {
        i2c_delay();
        if (!scl_high()) {
            return false;
        }
        i2c_delay();
        if (sda_read() != 0U) {
            result |= (uint8_t)(1U << bit);
        }
        scl_low();
    }

    if (send_ack) {
        sda_low();
    } else {
        sda_release();
    }
    i2c_delay();
    if (!scl_high()) {
        return false;
    }
    i2c_delay();
    scl_low();
    sda_release();
    *value = result;
    return true;
}

/* 总线恢复：释放SDA并输出9个SCL脉冲，再发送STOP。 */
static void i2c_recover_bus(void)
{
    uint32_t pulse;

    sda_release();
    for (pulse = 0U; pulse < 9U; pulse++) {
        scl_low();
        i2c_delay();
        (void)scl_high();
        i2c_delay();
    }
    i2c_stop();
}

/*
 * 手册标准读取语句：
 * START→写地址→命令→重复START→读地址→数据→NACK→STOP。
 */
static bool read_command(uint8_t command, uint8_t *data, uint32_t length)
{
    uint32_t index;

    if ((data == 0) || (length == 0U)) {
        last_error = 4U;
        return false;
    }

    last_error = 0U;
    if (!i2c_start()) {
        return false;
    }
    if (!i2c_write_byte((uint8_t)(NEWGRAY_I2C_ADDRESS << 1U))) {
        last_error = 1U;
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(command)) {
        last_error = 2U;
        i2c_stop();
        return false;
    }
    if (!i2c_start()) {
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte((uint8_t)((NEWGRAY_I2C_ADDRESS << 1U) | 1U))) {
        last_error = 3U;
        i2c_stop();
        return false;
    }
    for (index = 0U; index < length; index++) {
        /*
         * 多字节读取时，前面的字节由主机发送ACK，通知传感器继续发送；
         * 最后一个字节发送NACK，然后STOP结束本次命令读取。
         */
        if (!i2c_read_byte(&data[index], (index + 1U) < length)) {
            i2c_stop();
            return false;
        }
    }
    i2c_stop();
    return true;
}

static uint32_t logical_index(uint32_t sensor_index)
{
#if NEWGRAY_REVERSE_ORDER
    return (GRAY_SENSOR_COUNT - 1U) - sensor_index;
#else
    return sensor_index;
#endif
}

/*
 * 初始化PC2=SDA、PC3=SCL为开漏输出。内部弱上拉仅用于防止引脚悬空，
 * 正式接线仍必须外接到3.3V的上拉电阻，禁止使用传感器板5V上拉跳帽。
 */
bool NewGray_Init(void)
{
    uint32_t pin;
    uint32_t retry;
    uint8_t response;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    for (pin = NEWGRAY_SDA_PIN; pin <= NEWGRAY_SCL_PIN; pin++) {
        GPIOC->MODER = (GPIOC->MODER & ~(3UL << (pin * 2U))) |
                       (1UL << (pin * 2U));
        GPIOC->OTYPER |= (1UL << pin);
        GPIOC->OSPEEDR &= ~(3UL << (pin * 2U));
        GPIOC->PUPDR = (GPIOC->PUPDR & ~(3UL << (pin * 2U))) |
                       (1UL << (pin * 2U));
    }

    sda_release();
    scl_release();
    i2c_recover_bus();

    for (retry = 0U; retry < NEWGRAY_PING_RETRIES; retry++) {
        if (read_command(NEWGRAY_COMMAND_PING, &response, 1U) &&
            (response == NEWGRAY_PING_RESPONSE)) {
            return true;
        }
        i2c_recover_bus();
        Timebase_DelayMs(10U);
    }
    return false;
}

/* 读取0xDD数字位图，并转换成“黑线位=1”的统一循迹数据。 */
bool NewGray_Read(GraySensor_Data *data)
{
    uint8_t raw;
    uint32_t sensor;
    uint32_t index;
    bool is_white;

    if (data == 0) {
        last_error = 4U;
        return false;
    }
    if (!read_command(NEWGRAY_COMMAND_DIGITAL, &raw, 1U)) {
        return false;
    }

    data->raw_mask = raw;
    data->active_mask = 0U;
    data->active_count = 0U;

    for (sensor = 0U; sensor < GRAY_SENSOR_COUNT; sensor++) {
        index = logical_index(sensor);
        is_white = (raw & (1U << sensor)) != 0U;
        /*
         * 直接复用“循迹成功最终版”的已验证解释：
         * 0xDD的bit=1表示白色，展开为255；bit=0表示黑色，展开为0。
         */
        data->value[index] = is_white ? 255U : 0U;
        if (!is_white) {
            data->active_mask |= (uint8_t)(1U << index);
            data->active_count++;
        }
    }
    return true;
}

uint8_t NewGray_LastError(void)
{
    return last_error;
}
