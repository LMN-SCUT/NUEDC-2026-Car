#include "key.h"
#include "stm32f4xx.h"
#include "timebase.h"

/* H60 板载 K 键接 PE1，松开为高电平，按下为低电平。 */
void Key_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &gpio);
}

uint8_t Key_PressedEvent(void)
{
    static uint8_t released = 1U;

    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1) == Bit_RESET) {
        if (released != 0U) {
            DelayMs(25U);
            if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1) == Bit_RESET) {
                released = 0U;
                return 1U;
            }
        }
    } else {
        released = 1U;
    }
    return 0U;
}
