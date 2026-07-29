#include "key.h"
#include "timebase.h"
#include "stm32f4xx.h"

#define KEY_DEBOUNCE_MS 30U

static uint8_t last_raw_pressed;
static uint8_t stable_pressed;
static uint8_t press_event_armed;
static uint32_t raw_changed_ms;

/* 读取PE1当前原始电平：按下为1，松开为0。 */
static uint8_t read_start_key_raw(void)
{
    return ((GPIOE->IDR & (1UL << 1U)) == 0U) ? 1U : 0U;
}

/*
 * 初始化H60板载PE1用户按键和消抖状态。
 * 如果上电时按键已经按住，先不允许触发，必须松开后重新按下。
 */
void Key_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;

    GPIOE->MODER &= ~(3UL << (1U * 2U));
    GPIOE->PUPDR = (GPIOE->PUPDR & ~(3UL << (1U * 2U))) |
                   (1UL << (1U * 2U));

    last_raw_pressed = read_start_key_raw();
    stable_pressed = last_raw_pressed;
    press_event_armed = (stable_pressed == 0U) ? 1U : 0U;
    raw_changed_ms = Timebase_Millis();
}

/*
 * 非阻塞消抖和单次事件检测。
 * 主循环可以反复调用；只有一次新的、稳定的按下动作返回1。
 */
uint8_t Key_StartPressedEvent(void)
{
    uint8_t raw_pressed = read_start_key_raw();
    uint32_t now = Timebase_Millis();

    if (raw_pressed != last_raw_pressed) {
        last_raw_pressed = raw_pressed;
        raw_changed_ms = now;
    }

    if ((raw_pressed != stable_pressed) &&
        ((uint32_t)(now - raw_changed_ms) >= KEY_DEBOUNCE_MS)) {
        stable_pressed = raw_pressed;

        if (stable_pressed == 0U) {
            press_event_armed = 1U;
        } else if (press_event_armed != 0U) {
            press_event_armed = 0U;
            return 1U;
        }
    }

    return 0U;
}

/* 直接返回K按键(S4/PE1)电平，供串口确认按键焊接和引脚是否正常。 */
uint8_t Key_IsPressed(void)
{
    return read_start_key_raw();
}
