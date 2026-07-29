#include "motor.h"
#include "board_config.h"
#include "stm32f4xx.h"

static int16_t left_command;
static int16_t right_command;
static int16_t ma_command;
static int16_t mb_command;
static int16_t mc_command;
static int16_t md_command;

#if APP_ENABLE_MOTORS
static int16_t clamp_percent(int16_t value)
{
    if (value > 100) {
        return 100;
    }
    if (value < -100) {
        return -100;
    }
    return value;
}

static uint32_t percent_to_compare(int16_t percent, uint32_t period)
{
    uint32_t magnitude;
    if (percent < 0) {
        percent = (int16_t)-percent;
    }
    magnitude = (uint32_t)percent;
    return ((period + 1U) * magnitude) / 100U;
}
#endif

static void gpio_set_af(GPIO_TypeDef *port, uint32_t pin,
                        uint32_t alternate_function)
{
    uint32_t shift = pin * 2U;
    uint32_t afr_index = pin >> 3U;
    uint32_t afr_shift = (pin & 7U) * 4U;

    port->MODER = (port->MODER & ~(3UL << shift)) | (2UL << shift);
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << shift);
    port->PUPDR &= ~(3UL << shift);
    port->AFR[afr_index] =
        (port->AFR[afr_index] & ~(15UL << afr_shift)) |
        (alternate_function << afr_shift);
}

static void timer_pwm_common(TIM_TypeDef *timer, uint32_t period)
{
    timer->CR1 = 0U;
    timer->PSC = 0U;
    timer->ARR = period;
    timer->EGR = TIM_EGR_UG;
}

#if APP_ENABLE_MOTORS
/*
 * 控制AT8236的一个电机通道：
 * 正转时IN1输出PWM、IN2为0；反转时IN1为0、IN2输出PWM；0命令时双低。
 */
static void set_motor_channel(TIM_TypeDef *timer,
                              volatile uint32_t *input1_compare,
                              volatile uint32_t *input2_compare,
                              int16_t percent, uint8_t invert)
{
    uint32_t compare;

    percent = clamp_percent(percent);
    if (invert != 0U) {
        percent = (int16_t)-percent;
    }

    compare = percent_to_compare(percent, timer->ARR);
    if (percent > 0) {
        *input1_compare = compare;
        *input2_compare = 0U;
    } else if (percent < 0) {
        *input1_compare = 0U;
        *input2_compare = compare;
    } else {
        *input1_compare = 0U;
        *input2_compare = 0U;
    }
}
#endif

/*
 * 初始化H60四个板载电机接口：
 * MA/MB使用TIM1，MC使用TIM9，MD使用TIM12，PWM目标频率2 kHz。
 * 2 kHz与已成功运行的H60参考工程一致，也便于当前阶段测量PWM平均电压。
 * 初始化结束后立即Motor_Stop，避免上电误动作。
 */
void Motor_Init(void)
{
    uint32_t period;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    (void)RCC->AHB1ENR;

    /* MA/MB: TIM1 AF1 */
    gpio_set_af(GPIOE, 9U, 1U);
    gpio_set_af(GPIOE, 11U, 1U);
    gpio_set_af(GPIOE, 13U, 1U);
    gpio_set_af(GPIOE, 14U, 1U);
    /* MC: TIM9 AF3 */
    gpio_set_af(GPIOE, 5U, 3U);
    gpio_set_af(GPIOE, 6U, 3U);
    /* MD: TIM12 AF9 */
    gpio_set_af(GPIOB, 14U, 9U);
    gpio_set_af(GPIOB, 15U, 9U);

    period = (SystemCoreClock / MOTOR_PWM_FREQUENCY_HZ) - 1U;

    timer_pwm_common(TIM1, period);
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 |
                  TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC3PE |
                  TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 |
                  TIM_CCMR2_OC4PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    timer_pwm_common(TIM9, period);
    TIM9->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 |
                  TIM_CCMR1_OC2PE;
    TIM9->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM9->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    timer_pwm_common(TIM12, period);
    TIM12->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                   TIM_CCMR1_OC1PE |
                   TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 |
                   TIM_CCMR1_OC2PE;
    TIM12->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM12->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    left_command = 0;
    right_command = 0;
    ma_command = 0;
    mb_command = 0;
    mc_command = 0;
    md_command = 0;
    Motor_Stop();
}

/*
 * 左右侧速度统一入口。按H60板背面布局和实车接线：
 * MB/MD接收left_percent，MA/MC接收right_percent。
 * APP_ENABLE_MOTORS=0时仍会编译控制逻辑，但实际输出被强制保持为0。
 */
void Motor_SetSidePercent(int16_t left_percent, int16_t right_percent)
{
    Motor_SetWheelPercent(right_percent, left_percent,
                          right_percent, left_percent);
}

/*
 * 四轮独立PWM入口，供编码器速度PI使用。
 * left_command/right_command记录同侧较大的绝对命令，供堵转保护判断该侧
 * 是否正在受控运行；真正的四路占空比分别写入对应AT8236通道。
 */
void Motor_SetWheelPercent(int16_t ma_percent, int16_t mb_percent,
                           int16_t mc_percent, int16_t md_percent)
{
#if APP_ENABLE_MOTORS
    ma_percent = clamp_percent(ma_percent);
    mb_percent = clamp_percent(mb_percent);
    mc_percent = clamp_percent(mc_percent);
    md_percent = clamp_percent(md_percent);
    ma_command = ma_percent;
    mb_command = mb_percent;
    mc_command = mc_percent;
    md_command = md_percent;
    left_command = (mb_percent >= 0 ? mb_percent : -mb_percent) >=
                   (md_percent >= 0 ? md_percent : -md_percent) ?
                   mb_percent : md_percent;
    right_command = (ma_percent >= 0 ? ma_percent : -ma_percent) >=
                    (mc_percent >= 0 ? mc_percent : -mc_percent) ?
                    ma_percent : mc_percent;
    set_motor_channel(TIM1, &TIM1->CCR1, &TIM1->CCR2,
                      ma_percent, MOTOR_MA_INVERT);
    set_motor_channel(TIM1, &TIM1->CCR3, &TIM1->CCR4,
                      mb_percent, MOTOR_MB_INVERT);
    set_motor_channel(TIM9, &TIM9->CCR1, &TIM9->CCR2,
                      mc_percent, MOTOR_MC_INVERT);
    set_motor_channel(TIM12, &TIM12->CCR1, &TIM12->CCR2,
                      md_percent, MOTOR_MD_INVERT);
#else
    (void)ma_percent;
    (void)mb_percent;
    (void)mc_percent;
    (void)md_percent;
    left_command = 0;
    right_command = 0;
    ma_command = 0;
    mb_command = 0;
    mc_command = 0;
    md_command = 0;
    Motor_Stop();
#endif
}

/* 普通停车：四个AT8236通道双输入均为低，电机自由减速。 */
void Motor_Stop(void)
{
    left_command = 0;
    right_command = 0;
    ma_command = 0;
    mb_command = 0;
    mc_command = 0;
    md_command = 0;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
    TIM9->CCR1 = 0U;
    TIM9->CCR2 = 0U;
    TIM12->CCR1 = 0U;
    TIM12->CCR2 = 0U;
}

/* 主动短刹车：四个AT8236通道双输入均为高，用于终点快速停车。 */
void Motor_Brake(void)
{
    left_command = 0;
    right_command = 0;
#if APP_ENABLE_MOTORS
    TIM1->CCR1 = TIM1->ARR + 1U;
    TIM1->CCR2 = TIM1->ARR + 1U;
    TIM1->CCR3 = TIM1->ARR + 1U;
    TIM1->CCR4 = TIM1->ARR + 1U;
    TIM9->CCR1 = TIM9->ARR + 1U;
    TIM9->CCR2 = TIM9->ARR + 1U;
    TIM12->CCR1 = TIM12->ARR + 1U;
    TIM12->CCR2 = TIM12->ARR + 1U;
#else
    Motor_Stop();
#endif
}

/* 返回当前左侧有效命令；H60实车左侧为MB（左后）和MD（左前）。 */
int16_t Motor_LeftCommand(void)
{
    return left_command;
}

/* 返回当前右侧有效命令；H60实车右侧为MA（右后）和MC（右前）。 */
int16_t Motor_RightCommand(void)
{
    return right_command;
}

int16_t Motor_MACommand(void) { return ma_command; }
int16_t Motor_MBCommand(void) { return mb_command; }
int16_t Motor_MCCommand(void) { return mc_command; }
int16_t Motor_MDCommand(void) { return md_command; }
