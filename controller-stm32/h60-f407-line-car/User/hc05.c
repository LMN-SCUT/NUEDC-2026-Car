#include "hc05.h"
#include "board_config.h"
#include "stm32f4xx.h"

/* 根据当前APB1分频计算USART3实际外设时钟。 */
static uint32_t usart3_clock_hz(void)
{
    static const uint8_t apb_shift[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
    uint32_t ppre1 = (RCC->CFGR >> 10U) & 7U;
    return SystemCoreClock >> apb_shift[ppre1];
}

/*
 * HC-05串口初始化。选用扩展口空闲的USART3：
 * PB10为TX，PB11为RX，避免占用板载USB调试串口和电机/编码器资源。
 */
void HC05_Init(void)
{
    uint32_t pin;
    uint32_t afr_shift;
    uint32_t peripheral_clock;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->AHB1ENR;

    for (pin = 10U; pin <= 11U; pin++) {
        GPIOB->MODER = (GPIOB->MODER & ~(3UL << (pin * 2U))) |
                       (2UL << (pin * 2U));
        GPIOB->OTYPER &= ~(1UL << pin);
        GPIOB->OSPEEDR |= (2UL << (pin * 2U));
        GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (pin * 2U))) |
                       (1UL << (pin * 2U));
        afr_shift = (pin - 8U) * 4U;
        GPIOB->AFR[1] =
            (GPIOB->AFR[1] & ~(15UL << afr_shift)) |
            (7UL << afr_shift);
    }

    USART3->CR1 = 0U;
    USART3->CR2 = 0U;
    USART3->CR3 = 0U;
    peripheral_clock = usart3_clock_hz();
    USART3->BRR =
        (peripheral_clock + (HC05_BAUD_RATE / 2U)) / HC05_BAUD_RATE;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void HC05_SendByte(uint8_t data)
{
    while ((USART3->SR & USART_SR_TXE) == 0U) {
        __NOP();
    }
    USART3->DR = data;
}

void HC05_SendString(const char *text)
{
    if (text == 0) {
        return;
    }
    while (*text != '\0') {
        HC05_SendByte((uint8_t)*text);
        text++;
    }
}

void HC05_SendInt32(int32_t value)
{
    char digits[11];
    uint32_t magnitude;
    uint32_t count = 0U;

    if (value < 0) {
        HC05_SendByte((uint8_t)'-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[count++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(digits)));

    while (count > 0U) {
        HC05_SendByte((uint8_t)digits[--count]);
    }
}

void HC05_SendHex8(uint8_t value)
{
    static const char hexadecimal[] = "0123456789ABCDEF";
    HC05_SendByte((uint8_t)hexadecimal[(value >> 4U) & 0x0FU]);
    HC05_SendByte((uint8_t)hexadecimal[value & 0x0FU]);
}

uint8_t HC05_ByteAvailable(void)
{
    return ((USART3->SR & USART_SR_RXNE) != 0U) ? 1U : 0U;
}

uint8_t HC05_ReadByte(uint8_t *data)
{
    if ((data == 0) || (HC05_ByteAvailable() == 0U)) {
        return 0U;
    }
    *data = (uint8_t)USART3->DR;
    return 1U;
}
