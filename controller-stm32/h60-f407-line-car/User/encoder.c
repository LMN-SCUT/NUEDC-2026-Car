#include "encoder.h"
#include "board_config.h"
#include "stm32f4xx.h"

static void gpio_set_af_input(GPIO_TypeDef *port, uint32_t pin,
                              uint32_t alternate_function)
{
    uint32_t shift = pin * 2U;
    uint32_t afr_index = pin >> 3U;
    uint32_t afr_shift = (pin & 7U) * 4U;

    port->MODER = (port->MODER & ~(3UL << shift)) | (2UL << shift);
    port->PUPDR = (port->PUPDR & ~(3UL << shift)) | (1UL << shift);
    port->AFR[afr_index] =
        (port->AFR[afr_index] & ~(15UL << afr_shift)) |
        (alternate_function << afr_shift);
}

static void timer_encoder_init(TIM_TypeDef *timer, uint32_t maximum)
{
    timer->CR1 = 0U;
    timer->PSC = 0U;
    timer->ARR = maximum;
    timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                   TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 |
                   TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1;
    timer->CCER = 0U;
    timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    timer->CNT = 0U;
    timer->CR1 = TIM_CR1_CEN;
}

/*
 * 初始化四路硬件正交编码器：
 * MA=TIM2、MB=TIM3、MC=TIM5、MD=TIM4。
 * 定时器自动根据A/B相顺序加减计数，当前不假定哪个符号代表整车前进。
 */
void Encoder_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN |
                    RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN;
    (void)RCC->AHB1ENR;

    /* MA: PA15/PB3 TIM2 AF1 */
    gpio_set_af_input(GPIOA, 15U, 1U);
    gpio_set_af_input(GPIOB, 3U, 1U);
    /* MB: PB4/PB5 TIM3 AF2 */
    gpio_set_af_input(GPIOB, 4U, 2U);
    gpio_set_af_input(GPIOB, 5U, 2U);
    /* MC: PA0/PA1 TIM5 AF2 */
    gpio_set_af_input(GPIOA, 0U, 2U);
    gpio_set_af_input(GPIOA, 1U, 2U);
    /* MD: PD12/PD13 TIM4 AF2 */
    gpio_set_af_input(GPIOD, 12U, 2U);
    gpio_set_af_input(GPIOD, 13U, 2U);

    timer_encoder_init(TIM2, 0xFFFFFFFFUL);
    timer_encoder_init(TIM3, 0xFFFFUL);
    timer_encoder_init(TIM5, 0xFFFFFFFFUL);
    timer_encoder_init(TIM4, 0xFFFFUL);
}

/*
 * 读取一个控制周期内的四路脉冲增量并清零计数器。
 * 关中断保护用于保证“读取+清零”不会被其他中断打断。
 */
void Encoder_ReadAndReset(Encoder_Delta *delta)
{
    if (delta == 0) {
        return;
    }

    __disable_irq();
    delta->ma = (int32_t)TIM2->CNT;
    delta->mb = (int32_t)(int16_t)TIM3->CNT;
    delta->mc = (int32_t)TIM5->CNT;
    delta->md = (int32_t)(int16_t)TIM4->CNT;
    TIM2->CNT = 0U;
    TIM3->CNT = 0U;
    TIM5->CNT = 0U;
    TIM4->CNT = 0U;
    __enable_irq();

    /*
     * 按实机前进方向统一符号，方便后续四轮速度比较和PI闭环。
     * 堵转保护只看绝对值，不依赖该符号；里程累计则直接使用归一化结果。
     */
#if ENCODER_MA_INVERT
    delta->ma = -delta->ma;
#endif
#if ENCODER_MB_INVERT
    delta->mb = -delta->mb;
#endif
#if ENCODER_MC_INVERT
    delta->mc = -delta->mc;
#endif
#if ENCODER_MD_INVERT
    delta->md = -delta->md;
#endif
}
