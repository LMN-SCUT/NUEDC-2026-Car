#include "mpu6050.h"
#include <math.h>

// -------------------------- 1. 解决ulTicks链接错误 --------------------------
volatile unsigned long ulTicks = 0;

// -------------------------- 2. 软件I2C底层（PB6=SCL, PB7=SDA，时序超稳定） --------------------------
#define SCL_H() GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define SCL_L() GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define SDA_H() GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define SDA_L() GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define SDA_READ() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)

static void I2C_Delay(void)
{
    for(volatile u32 i=0; i<200; i++); // 拉长延时，保证稳定
}

static void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // 降速减少干扰
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SCL_H();
    SDA_H();
    I2C_Delay();
}

static void I2C_Start(void)
{
    SDA_H();
    SCL_H();
    I2C_Delay();
    SDA_L();
    I2C_Delay();
    SCL_L();
}

static void I2C_Stop(void)
{
    SDA_L();
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SDA_H();
    I2C_Delay();
}

static u8 I2C_Wait_Ack(void)
{
    u8 timeout = 0;
    SDA_H();
    I2C_Delay();
    SCL_H();
    I2C_Delay();

    while(SDA_READ())
    {
        timeout++;
        if(timeout > 200)
        {
            I2C_Stop();
            return 1;
        }
    }
    SCL_L();
    return 0;
}

static void I2C_Send_Byte(u8 dat)
{
    for(u8 i=0; i<8; i++)
    {
        if(dat & 0x80) SDA_H();
        else SDA_L();
        I2C_Delay();
        SCL_H();
        I2C_Delay();
        SCL_L();
        dat <<= 1;
    }
}

static u8 I2C_Read_Byte(u8 ack_en)
{
    u8 dat = 0;
    SDA_H();
    I2C_Delay();
    for(u8 i=0; i<8; i++)
    {
        dat <<= 1;
        SCL_H();
        I2C_Delay();
        if(SDA_READ()) dat |= 0x01;
        SCL_L();
        I2C_Delay();
    }
    if(ack_en) SDA_L();
    else SDA_H();
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SCL_L();
    SDA_H();
    I2C_Delay();
    return dat;
}

// -------------------------- 3. MPU6050/6500 寄存器读写 --------------------------
#define MPU_ADDR    0x68    // AD0接GND固定地址
#define WHO_AM_I    0x75
#define PWR_MGMT_1  0x6B
#define GYRO_ZOUT_H 0x47

static void MPU_Write_Reg(u8 reg, u8 dat)
{
    I2C_Start();
    I2C_Send_Byte(MPU_ADDR << 1);
    if(I2C_Wait_Ack() != 0) return;
    I2C_Send_Byte(reg);
    if(I2C_Wait_Ack() != 0) return;
    I2C_Send_Byte(dat);
    if(I2C_Wait_Ack() != 0) return;
    I2C_Stop();
    I2C_Delay();
}

static u8 MPU_Read_Reg(u8 reg)
{
    u8 dat;
    I2C_Start();
    I2C_Send_Byte(MPU_ADDR << 1);
    if(I2C_Wait_Ack() != 0) return 0xFF;
    I2C_Send_Byte(reg);
    if(I2C_Wait_Ack() != 0) return 0xFF;
    I2C_Stop();
    I2C_Delay();

    I2C_Start();
    I2C_Send_Byte((MPU_ADDR << 1) | 0x01);
    if(I2C_Wait_Ack() != 0) return 0xFF;
    dat = I2C_Read_Byte(0);
    I2C_Stop();
    I2C_Delay();
    return dat;
}

// -------------------------- 4. 核心功能变量 --------------------------
static float yaw_angle = 0;
static float gyro_z_offset = 0;
static uint8_t calibrated = 0;


// 初始化
void MPU6050_Init(void)
{
    MyI2C_Init();
    
    // 复位模块
    MPU_Write_Reg(PWR_MGMT_1, 0x80);
    for(volatile u32 i=0; i<1000000; i++);
    
    // 唤醒模块
    MPU_Write_Reg(PWR_MGMT_1, 0x00);
    for(volatile u32 i=0; i<500000; i++);
    
    // 配置陀螺仪量程±2000dps
    MPU_Write_Reg(0x1B, 0x18);
    
    // 初始化变量
    yaw_angle = 0;
    gyro_z_offset = 0;
    calibrated = 0;
}

// 获取偏航角
uint8_t MPU6050_DMP_Get_Yaw(float *yaw)
{
    // 第一次调用时快速校准零偏（小车静置）
    if(!calibrated)
    {
        int32_t sum = 0;
        for(u16 i=0; i<200; i++) // 减少校准次数，快速启动
        {
            u8 H = MPU_Read_Reg(GYRO_ZOUT_H);
            u8 L = MPU_Read_Reg(GYRO_ZOUT_H + 1);
            sum += (short)((H << 8) | L);
        }
        gyro_z_offset = sum / 200.0f;
        yaw_angle = 0;
        calibrated = 1;
    }
    
    // 读取陀螺仪Z轴数据
    u8 H = MPU_Read_Reg(GYRO_ZOUT_H);
    u8 L = MPU_Read_Reg(GYRO_ZOUT_H + 1);
    short gyro_raw = (short)((H << 8) | L);
    
    // 转换为角速度，减去零偏
    float gyro_z = (gyro_raw - gyro_z_offset) / 16.4f*1.13f;
    
    // 积分计算偏航角（dt=0.01s，对应10ms定时器中断）
    yaw_angle += gyro_z * 0.01f;
    
    // 角度归一化
    if(yaw_angle >=360) yaw_angle -= 360;
    if(yaw_angle < 0) yaw_angle += 360;
    
    *yaw = yaw_angle;
   return 0;
}
