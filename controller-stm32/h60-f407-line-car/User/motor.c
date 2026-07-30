#include "motor.h"
#include "board.h"
#include "stm32f4xx.h"
#include "timebase.h"

static uint16_t left_duty;
static uint16_t right_duty;
static int8_t left_direction;
static int8_t right_direction;

static void PwmChannel_Init(TIM_TypeDef *timer, uint8_t channel)
{
    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0U;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    if (channel == 1U) TIM_OC1Init(timer, &oc);
    if (channel == 2U) TIM_OC2Init(timer, &oc);
    if (channel == 3U) TIM_OC3Init(timer, &oc);
    if (channel == 4U) TIM_OC4Init(timer, &oc);
}

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef time;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM9, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_9 |
                    GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&time);
    time.TIM_CounterMode = TIM_CounterMode_Up;
    time.TIM_Period = MOTOR_PWM_TOP;
    time.TIM_Prescaler = 83U;  /* TIM1/TIM9: 168 MHz /84 /1000 = 2 kHz */
    TIM_TimeBaseInit(TIM1, &time);
    TIM_TimeBaseInit(TIM9, &time);
    time.TIM_Prescaler = 41U;  /* TIM12: 84 MHz /42 /1000 = 2 kHz */
    TIM_TimeBaseInit(TIM12, &time);

    PwmChannel_Init(TIM1, 1U);
    PwmChannel_Init(TIM1, 2U);
    PwmChannel_Init(TIM1, 3U);
    PwmChannel_Init(TIM1, 4U);
    PwmChannel_Init(TIM9, 1U);
    PwmChannel_Init(TIM9, 2U);
    PwmChannel_Init(TIM12, 1U);
    PwmChannel_Init(TIM12, 2U);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
    TIM_Cmd(TIM12, ENABLE);
    Motor_StopAll();
}

static uint16_t SpeedToDuty(float speed)
{
    float magnitude = (speed < 0.0f) ? -speed : speed;
    uint16_t duty;
    if (magnitude > TRACK_MAX_SPEED) magnitude = TRACK_MAX_SPEED;
    duty = (uint16_t)(magnitude * (float)MOTOR_PWM_TOP / TRACK_MAX_SPEED);
    if ((duty < TRACK_MIN_DUTY) && (magnitude > 1.0f)) duty = TRACK_MIN_DUTY;
    return duty;
}

/*
 * 物理轮位（用户实机确认）：
 * MA=右后(TIM1 CH1/2)，MB=左后(TIM1 CH3/4)，
 * MC=右前(TIM9 CH1/2)，MD=左前(TIM12 CH1/2)。
 * 前进电平（用户实机确认）：MA2、MB1、MC2、MD1 输出 PWM。
 */
void Motor_SetSides(float left_speed, float right_speed)
{
    uint16_t left = SpeedToDuty(left_speed);
    uint16_t right = SpeedToDuty(right_speed);
    int8_t next_left_direction;
    int8_t next_right_direction;
    uint8_t direction_changed;

    next_left_direction =
        (left == 0U) ? 0 : ((left_speed >= 0.0f) ? 1 : -1);
    next_right_direction =
        (right == 0U) ? 0 : ((right_speed >= 0.0f) ? 1 : -1);
    direction_changed = 0U;
    if ((left_direction != 0) && (next_left_direction != 0) &&
        (left_direction != next_left_direction)) {
        direction_changed = 1U;
    }
    if ((right_direction != 0) && (next_right_direction != 0) &&
        (right_direction != next_right_direction)) {
        direction_changed = 1U;
    }

    left_duty = left;
    right_duty = right;

    /*
     * 换向前先关闭每个 H 桥的正反两路，再打开目标方向。
     * 这样从正转切到反转时不会在两次寄存器写入之间短暂同时导通。
     */
    TIM1->CCR1 = 0U; TIM1->CCR2 = 0U;   /* MA 右后 */
    TIM9->CCR1 = 0U; TIM9->CCR2 = 0U;   /* MC 右前 */
    TIM1->CCR3 = 0U; TIM1->CCR4 = 0U;   /* MB 左后 */
    TIM12->CCR1 = 0U; TIM12->CCR2 = 0U; /* MD 左前 */

    /*
     * 仅在正反方向直接互换时保持 1 ms 全零输出。
     * 当前 PWM 为 2 kHz，1 ms 覆盖两个完整 PWM 周期，确保旧方向脉冲结束。
     * 从停止启动、同方向调速以及正常巡航都不会进入此延时。
     */
    if (direction_changed != 0U) {
        DelayMs(1U);
    }

    if (right_speed >= 0.0f) {
        TIM1->CCR2 = right; /* MA 右后 */
        TIM9->CCR2 = right; /* MC 右前 */
    } else {
        TIM1->CCR1 = right;
        TIM9->CCR1 = right;
    }

    if (left_speed >= 0.0f) {
        TIM1->CCR3 = left;   /* MB 左后 */
        TIM12->CCR1 = left;  /* MD 左前 */
    } else {
        TIM1->CCR4 = left;
        TIM12->CCR2 = left;
    }

    left_direction = next_left_direction;
    right_direction = next_right_direction;
}

void Motor_StopAll(void)
{
    TIM1->CCR1 = 0U; TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U; TIM1->CCR4 = 0U;
    TIM9->CCR1 = 0U; TIM9->CCR2 = 0U;
    TIM12->CCR1 = 0U; TIM12->CCR2 = 0U;
    left_duty = 0U;
    right_duty = 0U;
    left_direction = 0;
    right_direction = 0;
}

uint16_t Motor_LeftDuty(void) { return left_duty; }
uint16_t Motor_RightDuty(void) { return right_duty; }
