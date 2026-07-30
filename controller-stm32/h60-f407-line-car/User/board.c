#include "board.h"
#include "timebase.h"

static void Clock_Init(void)
{
    ErrorStatus hse_ready;

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    hse_ready = RCC_WaitForHSEStartUp();
    if (hse_ready != SUCCESS) {
        while (1) {
        }
    }

    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2);
    /* 与成功工程一致：PLLM=4、PLLN=168、PLLP=2、PLLQ=4。 */
    RCC_PLLConfig(RCC_PLLSource_HSE, 4U, 168U, 2U, 4U);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
    }

    FLASH_SetLatency(FLASH_Latency_5);
    FLASH_PrefetchBufferCmd(ENABLE);
    FLASH_InstructionCacheCmd(ENABLE);
    FLASH_DataCacheCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08U) {
    }
}

void Board_Init(void)
{
    GPIO_InitTypeDef gpio;

    Clock_Init();
    Timebase_Init();

    /* 板载蜂鸣器 PE0：用户实机已确认低电平为关闭。 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &gpio);
    GPIO_ResetBits(GPIOE, GPIO_Pin_0);
}
