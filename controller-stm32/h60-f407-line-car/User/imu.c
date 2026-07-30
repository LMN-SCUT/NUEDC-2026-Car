#include "imu.h"
#include "stm32f4xx.h"
#include "timebase.h"

ImuData imu;
static uint8_t imu_address = 0x6BU;
static float gyro_bias_z;

#define QMI_WHO_AM_I  0x00U
#define QMI_CTRL1     0x02U
#define QMI_CTRL2     0x03U
#define QMI_CTRL3     0x04U
#define QMI_CTRL7     0x08U
#define QMI_GYR_X_L   0x3BU
#define GYRO_SCALE    0.000266f

static uint8_t WaitEvent(uint32_t event)
{
    uint32_t timeout = 100000U;
    while (I2C_CheckEvent(I2C1, event) == ERROR) {
        if (timeout-- == 0U) return 0U;
    }
    return 1U;
}

void ImuBus_Init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    I2C_StructInit(&i2c);
    i2c.I2C_ClockSpeed = 400000U;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_Ack = I2C_Ack_Enable;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

static uint8_t ReadRegisters(uint8_t device, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t index;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0U;
    I2C_Send7bitAddress(I2C1, (uint8_t)(device << 1), I2C_Direction_Transmitter);
    if (!WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0U;
    I2C_SendData(I2C1, reg);
    if (!WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0U;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0U;
    I2C_Send7bitAddress(I2C1, (uint8_t)(device << 1), I2C_Direction_Receiver);
    if (!WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 0U;
    for (index = 0U; index < length; index++) {
        if (index + 1U == length) I2C_AcknowledgeConfig(I2C1, DISABLE);
        if (!WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return 0U;
        data[index] = I2C_ReceiveData(I2C1);
    }
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1U;
}

static uint8_t WriteRegister(uint8_t device, uint8_t reg, uint8_t value)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0U;
    I2C_Send7bitAddress(I2C1, (uint8_t)(device << 1), I2C_Direction_Transmitter);
    if (!WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0U;
    I2C_SendData(I2C1, reg);
    if (!WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0U;
    I2C_SendData(I2C1, value);
    if (!WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0U;
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 1U;
}

uint8_t Imu_Init(void)
{
    uint8_t candidates[2] = {0x6BU, 0x6AU};
    uint8_t who = 0U;
    uint8_t index;
    float sum = 0.0f;
    uint8_t buffer[6];
    uint16_t sample;

    for (index = 0U; index < 2U; index++) {
        if (ReadRegisters(candidates[index], QMI_WHO_AM_I, &who, 1U) && who == 0x05U) {
            imu_address = candidates[index];
            break;
        }
    }
    if (index == 2U) return 0U;

    if (!WriteRegister(imu_address, QMI_CTRL3, 0x44U)) return 0U;
    if (!WriteRegister(imu_address, QMI_CTRL2, 0x34U)) return 0U;
    if (!WriteRegister(imu_address, QMI_CTRL1, 0x40U)) return 0U;
    if (!WriteRegister(imu_address, QMI_CTRL7, 0x03U)) return 0U;
    DelayMs(50U);

    for (sample = 0U; sample < 500U; sample++) {
        if (ReadRegisters(imu_address, QMI_GYR_X_L, buffer, 6U)) {
            sum += (float)(int16_t)((buffer[5] << 8) | buffer[4]);
        }
        DelayMs(1U);
    }
    gyro_bias_z = (sum / 500.0f) * GYRO_SCALE;
    imu.ready = 1U;
    return 1U;
}

void Imu_Update(void)
{
    static uint32_t previous_ms;
    uint32_t now = Millis();
    float dt = (float)(now - previous_ms) * 0.001f;
    uint8_t buffer[6];
    int16_t raw;
    previous_ms = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    if (!ReadRegisters(imu_address, QMI_GYR_X_L, buffer, 6U)) {
        imu.error_count++;
        return;
    }
    raw = (int16_t)((buffer[5] << 8) | buffer[4]);
    imu.gyro_z = (float)raw * GYRO_SCALE - gyro_bias_z;
    if (imu.gyro_z > -0.003f && imu.gyro_z < 0.003f) imu.gyro_z = 0.0f;
    imu.yaw += imu.gyro_z * dt;
}
