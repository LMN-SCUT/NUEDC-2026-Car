#include "gray_sensor.h"
#include "board_config.h"
#include "timebase.h"
#include "stm32f4xx.h"

static const uint16_t gray_threshold[GRAY_SENSOR_COUNT] = {
    GRAY_THRESHOLD_0, GRAY_THRESHOLD_1,
    GRAY_THRESHOLD_2, GRAY_THRESHOLD_3,
    GRAY_THRESHOLD_4, GRAY_THRESHOLD_5,
    GRAY_THRESHOLD_6, GRAY_THRESHOLD_7
};

/* 根据0~7通道编号输出4051三位二进制地址。 */
static void select_channel(uint32_t channel)
{
    uint16_t set_mask = 0U;
    uint16_t reset_mask = 0U;

    if ((channel & 1U) != 0U) {
        set_mask |= (uint16_t)(1U << 4U);
    } else {
        reset_mask |= (uint16_t)(1U << 4U);
    }
    if ((channel & 2U) != 0U) {
        set_mask |= (uint16_t)(1U << 5U);
    } else {
        reset_mask |= (uint16_t)(1U << 5U);
    }
    if ((channel & 4U) != 0U) {
        set_mask |= (uint16_t)(1U << 6U);
    } else {
        reset_mask |= (uint16_t)(1U << 6U);
    }
    GPIOA->BSRRL = set_mask;
    GPIOA->BSRRH = reset_mask;
}

/* 启动一次ADC1规则转换；超时返回false，防止程序永久卡死。 */
static bool read_adc(uint16_t *value)
{
    uint32_t timeout = SystemCoreClock / 1000U;

    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    *value = (uint16_t)ADC1->DR;
    return true;
}

/* 将4051电气通道编号转换为车体从左到右的数组下标。 */
static uint32_t result_index(uint32_t channel)
{
#if GRAY_REVERSE_ORDER
    return (GRAY_SENSOR_COUNT - 1U) - channel;
#else
    return channel;
#endif
}

/* 按当前黑线极性和该探头阈值判断是否压到黑线。 */
static bool is_on_line(uint16_t value, uint16_t threshold)
{
#if GRAY_LINE_IS_DARK
    return value < threshold;
#else
    return value > threshold;
#endif
}

/*
 * 初始化4051控制线和PC2模拟输入。
 * PA3输出低电平使能灰度板，PA4/5/6选择通道，ADC1读取PC2。
 */
void GraySensor_Init(void)
{
    uint32_t pin;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->AHB1ENR;

    for (pin = 3U; pin <= 6U; pin++) {
        GPIOA->MODER = (GPIOA->MODER & ~(3UL << (pin * 2U))) |
                       (1UL << (pin * 2U));
        GPIOA->OTYPER &= ~(1UL << pin);
        GPIOA->OSPEEDR |= (2UL << (pin * 2U));
        GPIOA->PUPDR &= ~(3UL << (pin * 2U));
    }
    /* PC2 = ADC1_IN12，模拟输入、无上下拉。 */
    GPIOC->MODER |= (3UL << (2U * 2U));
    GPIOC->PUPDR &= ~(3UL << (2U * 2U));

    /* ADC 公共时钟 PCLK2/4；12位、单次转换、通道12。 */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
    ADC1->CR1 = 0U;
    ADC1->CR2 = ADC_CR2_EOCS;
    ADC1->SQR1 = 0U;
    ADC1->SQR3 = 12U;
    ADC1->SMPR1 = (ADC1->SMPR1 & ~(7UL << 6U)) | (6UL << 6U);
    ADC1->CR2 |= ADC_CR2_ADON;

    /* EN低有效。 */
    GPIOA->BSRRH = (uint16_t)(1U << 3U);
    select_channel(0U);
}

/*
 * 完成一帧八路灰度读取：
 * 每次换路先等待输出稳定，再连续采样多次取平均，最后生成黑线位图。
 * 返回false时调用方必须停车，不能沿用上一帧电机命令。
 */
bool GraySensor_Read(GraySensor_Data *data)
{
    uint32_t channel;
    uint32_t sample;
    uint32_t sum;
    uint32_t index;
    uint16_t value;

    if (data == 0) {
        return false;
    }

    data->active_mask = 0U;
    data->active_count = 0U;

    for (channel = 0U; channel < GRAY_SENSOR_COUNT; channel++) {
        select_channel(channel);
        Timebase_DelayMs(GRAY_MUX_SETTLE_MS);

        sum = 0U;
        for (sample = 0U; sample < GRAY_SAMPLES_PER_CHANNEL; sample++) {
            if (!read_adc(&value)) {
                return false;
            }
            sum += value;
        }

        index = result_index(channel);
        data->value[index] =
            (uint16_t)(sum / GRAY_SAMPLES_PER_CHANNEL);
    }

    for (index = 0U; index < GRAY_SENSOR_COUNT; index++) {
        if (is_on_line(data->value[index], gray_threshold[index])) {
            data->active_mask |= (uint8_t)(1U << index);
            data->active_count++;
        }
    }
    return true;
}
