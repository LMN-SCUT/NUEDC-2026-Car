/* MG996R 台架测试：MSPM0G3507 LaunchPad
 * PWM 输出：PA12 / TIMG0-C0（LaunchPad J4.8）
 * 舵机供电必须使用独立 5~6 V 电源，TI 与舵机共地。
 *
 * 先用 CCS Watch 修改 g_servo_pulse_ticks：
 *   188 ~= 1500 us 中位，125 ~= 1000 us，250 ~= 2000 us
 *   仅在确认安全后再扩大范围，禁止一开始使用极限脉宽。
 */
#include "ti_msp_dl_config.h"
#include <stdint.h>

volatile uint16_t g_servo_pulse_ticks = 188U;
volatile uint16_t g_servo_min_ticks = 125U;
volatile uint16_t g_servo_max_ticks = 250U;
volatile uint32_t g_test_seconds;
volatile uint32_t g_pwm_updates;
volatile uint8_t g_test_running = 1U;

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(32000U);
    }
}

static uint16_t clamp_ticks(uint16_t ticks)
{
    if (ticks < 100U) {
        return 100U;
    }
    if (ticks > 275U) {
        return 275U;
    }
    return ticks;
}

int main(void)
{
    uint32_t elapsed_ms = 0U;
    SYSCFG_DL_init();
    DL_TimerG_startCounter(SERVO_PWM_INST);

    /* 上电先保持中位，给操作者留出接线确认时间。 */
    servo_set(188U);
    delay_ms(1000U);

    while (1) {
        uint16_t pulse = clamp_ticks(g_servo_pulse_ticks);
        servo_set(pulse);
        g_pwm_updates++;

        /* LED2/板载用户灯：固件运行心跳；灯脚若与板型不同不影响 PWM。 */
        if ((elapsed_ms % 500U) == 0U) {
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_26);
        }
        elapsed_ms++;
        if (elapsed_ms >= 1000U) {
            elapsed_ms = 0U;
            g_test_seconds++;
        }
        delay_ms(1U);
    }
}
