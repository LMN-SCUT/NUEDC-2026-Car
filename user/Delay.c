/*
 * 文件名：Delay.c
 * 功能：延时模块实现
 */
#include "Delay.h"

static uint32_t tick = 0;  // 全局时基变量（ms）


void Delay_Init(void)
{
    // SysTick时钟=72MHz，配置为72000次中断一次，即1ms
    SysTick_Config(SystemCoreClock / 1000);
}


void Delay_TickInc(void)
{
    tick++;
}


uint32_t Delay_GetTick(void)
{
    return tick;
}


void Delay_ms(uint32_t ms)
{
    uint32_t start = Delay_GetTick();
    while((Delay_GetTick() - start) < ms);  // 等待时间差达到目标
}


void Delay_s(uint32_t s)
{
    while(s--)
    {
        Delay_ms(1000);  // 1s=1000ms
    }
}
