#include "timebase.h"
#include "stm32f4xx.h"

static volatile uint32_t system_milliseconds;

/* SysTick每1 ms调用一次，只负责累加系统时间，不在中断内执行控制算法。 */
void SysTick_Handler(void)
{
    system_milliseconds++;
}

/* 启动1 ms系统时基；其他模块的超时和控制周期都依赖本接口。 */
void Timebase_Init(void)
{
    SystemCoreClockUpdate();
    system_milliseconds = 0U;
    (void)SysTick_Config(SystemCoreClock / 1000U);
}

/* 返回系统毫秒数。调用方应使用无符号减法计算时间差，以兼容回绕。 */
uint32_t Timebase_Millis(void)
{
    return system_milliseconds;
}

/* 毫秒级阻塞延时，目前用于4051换路后的模拟量建立等待。 */
void Timebase_DelayMs(uint32_t milliseconds)
{
    uint32_t start = Timebase_Millis();
    while ((uint32_t)(Timebase_Millis() - start) < milliseconds) {
        __NOP();
    }
}
