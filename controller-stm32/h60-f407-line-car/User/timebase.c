#include "timebase.h"
#include "stm32f4xx.h"

static volatile uint32_t tick_ms;

/* SysTick 每 1 ms 进入一次中断，供所有模块统一计时。 */
void Timebase_Init(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);
}

void SysTick_Handler(void)
{
    tick_ms++;
}

uint32_t Millis(void)
{
    return tick_ms;
}

void DelayMs(uint32_t ms)
{
    uint32_t start = Millis();
    while ((uint32_t)(Millis() - start) < ms) {
    }
}

/* 只用于软件 I2C；循环次数按 168 MHz Cortex-M4 实测前的保守值设置。 */
void DelayUs(uint32_t us)
{
    volatile uint32_t count;
    while (us--) {
        count = 42U;
        while (count--) {
            __NOP();
        }
    }
}
