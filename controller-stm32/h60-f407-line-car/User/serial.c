#include "serial.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

/* HC-05：USART3_TX=PB10，USART3_RX=PB11，115200-8-N-1。 */
void Serial_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200U;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &usart);
    USART_Cmd(USART3, ENABLE);
}

static void Serial_Write(const char *text)
{
    while (*text != '\0') {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(USART3, (uint16_t)(uint8_t)*text++);
    }
}

void Serial_Printf(const char *format, ...)
{
    char buffer[180];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial_Write(buffer);
}
