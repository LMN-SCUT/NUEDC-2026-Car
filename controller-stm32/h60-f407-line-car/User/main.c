#include "stm32f4xx.h"
#include "board_config.h"
#include "timebase.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "speed_pi.h"
#include "motor_protection.h"
#include "newgrey.h"
#include "line_follower.h"
#include "hc05.h"

typedef enum {
    APP_WAIT_START = 0,
    APP_RUNNING,
    APP_FINISHED,
    APP_ERROR
} App_State;

volatile App_State app_state = APP_WAIT_START;
volatile uint32_t run_time_ms;
volatile uint16_t gray_raw[GRAY_SENSOR_COUNT];
volatile uint8_t gray_active_mask;
volatile uint8_t gray_active_count;
volatile uint8_t gray_i2c_raw_mask;
volatile uint8_t gray_i2c_error;
volatile uint8_t gray_i2c_online;
volatile float line_error;
volatile Encoder_Delta encoder_delta_debug;
volatile LineFollower_Status line_status;

#if APP_MOTOR_TEST_MODE && ENCODER_PI_TEST_ENABLE
/* 判断10秒累计速度是否落在目标的±10%内，只作为闭环初步验收标准。 */
static uint8_t pi_total_within_10_percent(int32_t actual, int32_t target)
{
    int32_t minimum = (target * 90) / 100;
    int32_t maximum = (target * 110) / 100;
    return ((actual >= minimum) && (actual <= maximum)) ? 1U : 0U;
}
#endif

/*
 * 实机已经确认PE0高电平会让板载蜂鸣器持续鸣叫，与H60原理图一致。
 * 因此正常运行时将PE0配置为推挽输出低电平，可靠关闭蜂鸣器。
 */
static void board_buzzer_force_off(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;
    GPIOE->BSRRH = (uint16_t)(1U << 0U);
    GPIOE->MODER = (GPIOE->MODER & ~(3UL << (0U * 2U))) |
                   (1UL << (0U * 2U));
    GPIOE->OTYPER &= ~(1UL << 0U);
    GPIOE->PUPDR &= ~(3UL << (0U * 2U));
}

/* 将本帧灰度数据复制到全局变量，便于在Keil Watch中观察和标定。 */
#if !APP_MOTOR_TEST_MODE
static void publish_gray_debug(const GraySensor_Data *gray)
{
    uint32_t index;
    for (index = 0U; index < GRAY_SENSOR_COUNT; index++) {
        gray_raw[index] = gray->value[index];
    }
    gray_active_mask = gray->active_mask;
    gray_active_count = gray->active_count;
    gray_i2c_raw_mask = gray->raw_mask;
}
#endif

/*
 * 灰度测试遥测：每秒发送一行ASCII文本：
 * GRAY,在线状态,I2C错误码,D数字位图,S1..S8,B黑线位图,N见黑数,E误差x100,ST状态
 * S1~S8直接按成功参考工程的方法，把0xDD的8个位展开为白255、黑0。
 * 在线状态为0时先查看错误码，不应把尚未更新的八路0值误认为有效灰度。
 */
#if HC05_TEST_STREAM_ENABLE && !APP_ENABLE_MOTORS
static void send_gray_test_frame(void)
{
    uint32_t index;

    HC05_SendString("GRAY,");
    HC05_SendInt32((int32_t)gray_i2c_online);
    HC05_SendString(",");
    HC05_SendInt32((int32_t)gray_i2c_error);
    HC05_SendString(",D=0x");
    HC05_SendHex8(gray_i2c_raw_mask);
    for (index = 0U; index < GRAY_SENSOR_COUNT; index++) {
        HC05_SendString(",");
        HC05_SendInt32((int32_t)gray_raw[index]);
    }
    HC05_SendString(",B=0x");
    HC05_SendHex8(gray_active_mask);
    HC05_SendString(",N=");
    HC05_SendInt32((int32_t)gray_active_count);
    HC05_SendString(",E=");
    HC05_SendInt32((int32_t)(line_error * 100.0f));
    HC05_SendString(",ST=");
    HC05_SendInt32((int32_t)line_status);
    HC05_SendString("\r\n");
}
#endif

/*
 * 应用主入口。
 *
 * APP_WAIT_START：持续停车、采样并等待PE1按键；
 * APP_RUNNING：周期读取灰度和编码器，只执行环形循迹，满30秒无条件停止；
 * APP_FINISHED：30秒到达后保持短刹车，不使用A点启停线判断；
 * APP_ERROR：ADC异常或持续丢线后保持停车。
 */
int main(void)
{
    uint32_t last_control_ms = 0U;
#if HC05_TEST_STREAM_ENABLE && !APP_ENABLE_MOTORS
    uint32_t last_test_stream_ms = 0U;
#endif
#if APP_MOTOR_TEST_MODE
    uint32_t last_motor_test_report_ms = 0U;
    int32_t motor_test_encoder_ma = 0;
    int32_t motor_test_encoder_mb = 0;
    int32_t motor_test_encoder_mc = 0;
    int32_t motor_test_encoder_md = 0;
    int32_t encoder_total_ma = 0;
    int32_t encoder_total_mb = 0;
    int32_t encoder_total_mc = 0;
    int32_t encoder_total_md = 0;
    uint8_t encoder_pass_mask = 0U;
    const SpeedPI_Status *speed_pi_status;
#endif
    uint32_t run_started_ms = 0U;
#if !APP_MOTOR_TEST_MODE
    GraySensor_Data gray;
#endif
    Encoder_Delta encoder_delta;

    Timebase_Init();
    board_buzzer_force_off();
    HC05_Init();
#if APP_MOTOR_TEST_MODE
#if ENCODER_PI_TEST_ENABLE
    HC05_SendString("\r\nENCODER PI TEST READY,LIFT WHEELS,PRESS KEY\r\n");
    HC05_SendString("TARGET=70 COUNTS/20MS,REPORT=200MS\r\n");
#else
    HC05_SendString("\r\nENCODER TEST READY,LIFT WHEELS,PRESS KEY\r\n");
#endif
    HC05_SendString("OK bits: bit0=MA,bit1=MB,bit2=MC,bit3=MD\r\n");
#else
    HC05_SendString("\r\nSTM32F407 8CH GRAY 115200\r\n");
#endif
    Key_Init();
    Motor_Init();
    Motor_Stop();
    Encoder_Init();
    SpeedPI_Init();
    MotorProtection_Init(Timebase_Millis());
#if APP_MOTOR_TEST_MODE
    /* 电机验证与灰度无关，跳过I2C握手，避免灰度未接时阻塞测试。 */
    gray_i2c_online = 0U;
    gray_i2c_error = 0U;
#else
    gray_i2c_online = NewGray_Init() ? 1U : 0U;
    gray_i2c_error = NewGray_LastError();
#endif
    LineFollower_Init();

    while (1) {
        if ((uint32_t)(Timebase_Millis() - last_control_ms) <
            APP_CONTROL_PERIOD_MS) {
            continue;
        }
        last_control_ms = Timebase_Millis();

#if APP_MOTOR_TEST_MODE
        /*
         * 四电机正命令验证：
         * PE1按下后，左右两侧统一给正40%命令，运行10秒后普通停车并锁定。
         * 这里的“正”是软件命令正方向；四个轮子的物理前进方向必须架空观察。
         * 当前为纯电机驱动隔离测试，临时关闭编码器堵转判定；正式循迹仍保留保护。
         */
        Encoder_ReadAndReset(&encoder_delta);
        encoder_delta_debug = encoder_delta;

        switch (app_state) {
            case APP_WAIT_START:
                Motor_Stop();
                run_time_ms = 0U;
                if ((uint32_t)(Timebase_Millis() -
                               last_motor_test_report_ms) >= 1000U) {
                    last_motor_test_report_ms = Timebase_Millis();
                    HC05_SendString("WAIT,K=");
                    HC05_SendInt32((int32_t)Key_IsPressed());
                    HC05_SendString("\r\n");
                }
                if (Key_StartPressedEvent() != 0U) {
                    run_started_ms = Timebase_Millis();
                    last_motor_test_report_ms = run_started_ms;
                    motor_test_encoder_ma = 0;
                    motor_test_encoder_mb = 0;
                    motor_test_encoder_mc = 0;
                    motor_test_encoder_md = 0;
                    encoder_total_ma = 0;
                    encoder_total_mb = 0;
                    encoder_total_mc = 0;
                    encoder_total_md = 0;
                    encoder_pass_mask = 0U;
                    MotorProtection_Reset(Timebase_Millis());
#if ENCODER_PI_TEST_ENABLE
                    SpeedPI_Reset();
                    SpeedPI_SetTargets(ENCODER_PI_TEST_TARGET,
                                       ENCODER_PI_TEST_TARGET,
                                       ENCODER_PI_TEST_TARGET,
                                       ENCODER_PI_TEST_TARGET);
#endif
                    app_state = APP_RUNNING;
#if ENCODER_PI_TEST_ENABLE
                    HC05_SendString("KEY_OK,PI_CLOSED_LOOP_START,10S\r\n");
#else
                    HC05_SendString("KEY_OK,ENCODER_TEST_START,40_PERCENT,10S\r\n");
#endif
                }
                break;

            case APP_RUNNING:
                run_time_ms =
                    (uint32_t)(Timebase_Millis() - run_started_ms);
                /*
                 * 汇总最近1秒四路硬件编码器的有符号增量。
                 * 正负号只表示A/B相顺序，当前阶段重点先确认四路绝对值都不为0。
                 */
                motor_test_encoder_ma += encoder_delta.ma;
                motor_test_encoder_mb += encoder_delta.mb;
                motor_test_encoder_mc += encoder_delta.mc;
                motor_test_encoder_md += encoder_delta.md;
                encoder_total_ma += encoder_delta.ma;
                encoder_total_mb += encoder_delta.mb;
                encoder_total_mc += encoder_delta.mc;
                encoder_total_md += encoder_delta.md;
                if (run_time_ms >= APP_MOTOR_TEST_MS) {
#if ENCODER_PI_TEST_ENABLE
                    SpeedPI_Reset();
#else
                    Motor_Stop();
#endif
                    app_state = APP_FINISHED;
                    /*
                     * 最终判定按10秒累计绝对计数完成。门限乘以测试秒数，
                     * 防止某一路只偶发跳动几下也被误认为编码器正常。
                     */
                    encoder_pass_mask = 0U;
                    if ((encoder_total_ma >=
                         (ENCODER_TEST_MIN_COUNTS_1S *
                          (int32_t)(APP_MOTOR_TEST_MS / 1000U))) ||
                        (encoder_total_ma <=
                         -(ENCODER_TEST_MIN_COUNTS_1S *
                           (int32_t)(APP_MOTOR_TEST_MS / 1000U)))) {
                        encoder_pass_mask |= 0x01U;
                    }
                    if ((encoder_total_mb >=
                         (ENCODER_TEST_MIN_COUNTS_1S *
                          (int32_t)(APP_MOTOR_TEST_MS / 1000U))) ||
                        (encoder_total_mb <=
                         -(ENCODER_TEST_MIN_COUNTS_1S *
                           (int32_t)(APP_MOTOR_TEST_MS / 1000U)))) {
                        encoder_pass_mask |= 0x02U;
                    }
                    if ((encoder_total_mc >=
                         (ENCODER_TEST_MIN_COUNTS_1S *
                          (int32_t)(APP_MOTOR_TEST_MS / 1000U))) ||
                        (encoder_total_mc <=
                         -(ENCODER_TEST_MIN_COUNTS_1S *
                           (int32_t)(APP_MOTOR_TEST_MS / 1000U)))) {
                        encoder_pass_mask |= 0x04U;
                    }
                    if ((encoder_total_md >=
                         (ENCODER_TEST_MIN_COUNTS_1S *
                          (int32_t)(APP_MOTOR_TEST_MS / 1000U))) ||
                        (encoder_total_md <=
                         -(ENCODER_TEST_MIN_COUNTS_1S *
                           (int32_t)(APP_MOTOR_TEST_MS / 1000U)))) {
                        encoder_pass_mask |= 0x08U;
                    }

                    HC05_SendString("FINAL,TMA=");
                    HC05_SendInt32(encoder_total_ma);
                    HC05_SendString(",TMB=");
                    HC05_SendInt32(encoder_total_mb);
                    HC05_SendString(",TMC=");
                    HC05_SendInt32(encoder_total_mc);
                    HC05_SendString(",TMD=");
                    HC05_SendInt32(encoder_total_md);
                    HC05_SendString(",OK=0x");
                    HC05_SendHex8(encoder_pass_mask);
                    HC05_SendString("\r\nRESULT,MA=");
                    HC05_SendString((encoder_pass_mask & 0x01U) ?
                                    "PASS" : "FAIL");
                    HC05_SendString(",MB=");
                    HC05_SendString((encoder_pass_mask & 0x02U) ?
                                    "PASS" : "FAIL");
                    HC05_SendString(",MC=");
                    HC05_SendString((encoder_pass_mask & 0x04U) ?
                                    "PASS" : "FAIL");
                    HC05_SendString(",MD=");
                    HC05_SendString((encoder_pass_mask & 0x08U) ?
                                    "PASS" : "FAIL");
                    HC05_SendString("\r\n");
                    if (encoder_pass_mask == 0x0FU) {
                        HC05_SendString("ENCODER_TEST_PASS,ALL_4_OK\r\n");
                    } else {
                        HC05_SendString("ENCODER_TEST_FAIL,CHECK_WIRING\r\n");
                    }
#if ENCODER_PI_TEST_ENABLE
                    {
                        int32_t pi_target_total =
                            (int32_t)(ENCODER_PI_TEST_TARGET *
                            (int32_t)(APP_MOTOR_TEST_MS /
                                      APP_CONTROL_PERIOD_MS));
                        uint8_t pi_pass_ma =
                            pi_total_within_10_percent(encoder_total_ma,
                                                       pi_target_total);
                        uint8_t pi_pass_mb =
                            pi_total_within_10_percent(encoder_total_mb,
                                                       pi_target_total);
                        uint8_t pi_pass_mc =
                            pi_total_within_10_percent(encoder_total_mc,
                                                       pi_target_total);
                        uint8_t pi_pass_md =
                            pi_total_within_10_percent(encoder_total_md,
                                                       pi_target_total);
                    HC05_SendString("PI_TARGET_TOTAL=");
                        HC05_SendInt32(pi_target_total);
                        HC05_SendString("\r\nPI_10PCT,MA=");
                        HC05_SendString(pi_pass_ma ? "PASS" : "FAIL");
                        HC05_SendString(",MB=");
                        HC05_SendString(pi_pass_mb ? "PASS" : "FAIL");
                        HC05_SendString(",MC=");
                        HC05_SendString(pi_pass_mc ? "PASS" : "FAIL");
                        HC05_SendString(",MD=");
                        HC05_SendString(pi_pass_md ? "PASS" : "FAIL");
                        HC05_SendString("\r\n");
                        if (pi_pass_ma && pi_pass_mb &&
                            pi_pass_mc && pi_pass_md) {
                            HC05_SendString("SPEED_PI_TEST_PASS\r\n");
                        } else {
                            HC05_SendString("SPEED_PI_NEEDS_TUNING\r\n");
                        }
                    }
#endif
                    break;
                }

#if ENCODER_PI_TEST_ENABLE
                /*
                 * 闭环测试中PI是唯一PWM写入者：目标固定为每20 ms 70计数，
                 * 四路根据各自编码器反馈独立调节占空比。
                 */
                SpeedPI_Update(&encoder_delta);
#else
                Motor_SetSidePercent(APP_MOTOR_TEST_PERCENT,
                                     APP_MOTOR_TEST_PERCENT);
#endif
                if ((uint32_t)(Timebase_Millis() -
                               last_motor_test_report_ms) >=
#if ENCODER_PI_TEST_ENABLE
                    ENCODER_PI_TEST_REPORT_MS
#else
                    1000U
#endif
                    ) {
                    last_motor_test_report_ms = Timebase_Millis();
#if ENCODER_PI_TEST_ENABLE
                    speed_pi_status = SpeedPI_GetStatus();
                    HC05_SendString("PI,MS=");
                    HC05_SendInt32((int32_t)run_time_ms);
                    HC05_SendString(",T=");
                    HC05_SendInt32((int32_t)(ENCODER_PI_TEST_TARGET *
                        (int32_t)(ENCODER_PI_TEST_REPORT_MS /
                                  APP_CONTROL_PERIOD_MS)));
                    HC05_SendString(",MA=");
                    HC05_SendInt32(motor_test_encoder_ma);
                    HC05_SendString(",MB=");
                    HC05_SendInt32(motor_test_encoder_mb);
                    HC05_SendString(",MC=");
                    HC05_SendInt32(motor_test_encoder_mc);
                    HC05_SendString(",MD=");
                    HC05_SendInt32(motor_test_encoder_md);
                    HC05_SendString(",PA=");
                    HC05_SendInt32(speed_pi_status->pwm_ma);
                    HC05_SendString(",PB=");
                    HC05_SendInt32(speed_pi_status->pwm_mb);
                    HC05_SendString(",PC=");
                    HC05_SendInt32(speed_pi_status->pwm_mc);
                    HC05_SendString(",PD=");
                    HC05_SendInt32(speed_pi_status->pwm_md);
                    HC05_SendString("\r\n");
#else
                    HC05_SendString("RUN,MS=");
                    HC05_SendInt32((int32_t)run_time_ms);
                    HC05_SendString(",ARR=");
                    HC05_SendInt32((int32_t)TIM1->ARR);
                    HC05_SendString(",MA1=");
                    HC05_SendInt32((int32_t)TIM1->CCR1);
                    HC05_SendString(",MA2=");
                    HC05_SendInt32((int32_t)TIM1->CCR2);
                    HC05_SendString(",MB1=");
                    HC05_SendInt32((int32_t)TIM1->CCR3);
                    HC05_SendString(",MB2=");
                    HC05_SendInt32((int32_t)TIM1->CCR4);
                    HC05_SendString(",MC1=");
                    HC05_SendInt32((int32_t)TIM9->CCR1);
                    HC05_SendString(",MC2=");
                    HC05_SendInt32((int32_t)TIM9->CCR2);
                    HC05_SendString(",MD1=");
                    HC05_SendInt32((int32_t)TIM12->CCR1);
                    HC05_SendString(",MD2=");
                    HC05_SendInt32((int32_t)TIM12->CCR2);
                    HC05_SendString(",EMA=");
                    HC05_SendInt32(motor_test_encoder_ma);
                    HC05_SendString(",EMB=");
                    HC05_SendInt32(motor_test_encoder_mb);
                    HC05_SendString(",EMC=");
                    HC05_SendInt32(motor_test_encoder_mc);
                    HC05_SendString(",EMD=");
                    HC05_SendInt32(motor_test_encoder_md);
                    encoder_pass_mask = 0U;
                    if ((motor_test_encoder_ma >=
                         ENCODER_TEST_MIN_COUNTS_1S) ||
                        (motor_test_encoder_ma <=
                         -ENCODER_TEST_MIN_COUNTS_1S)) {
                        encoder_pass_mask |= 0x01U;
                    }
                    if ((motor_test_encoder_mb >=
                         ENCODER_TEST_MIN_COUNTS_1S) ||
                        (motor_test_encoder_mb <=
                         -ENCODER_TEST_MIN_COUNTS_1S)) {
                        encoder_pass_mask |= 0x02U;
                    }
                    if ((motor_test_encoder_mc >=
                         ENCODER_TEST_MIN_COUNTS_1S) ||
                        (motor_test_encoder_mc <=
                         -ENCODER_TEST_MIN_COUNTS_1S)) {
                        encoder_pass_mask |= 0x04U;
                    }
                    if ((motor_test_encoder_md >=
                         ENCODER_TEST_MIN_COUNTS_1S) ||
                        (motor_test_encoder_md <=
                         -ENCODER_TEST_MIN_COUNTS_1S)) {
                        encoder_pass_mask |= 0x08U;
                    }
                    HC05_SendString(",OK=0x");
                    HC05_SendHex8(encoder_pass_mask);
                    HC05_SendString("\r\n");
#endif
                    motor_test_encoder_ma = 0;
                    motor_test_encoder_mb = 0;
                    motor_test_encoder_mc = 0;
                    motor_test_encoder_md = 0;
                }
#if APP_MOTOR_TEST_STALL_CHECK
                if (MotorProtection_Update(Timebase_Millis(),
                                           &encoder_delta)) {
                    Motor_Stop();
                    app_state = APP_ERROR;
                    HC05_SendString("STALL_OR_ENCODER_ERROR,STOP\r\n");
                }
#endif
                break;

            case APP_FINISHED:
            case APP_ERROR:
            default:
                Motor_Stop();
                break;
        }
        continue;
#else

        if (!NewGray_Read(&gray)) {
            Motor_Stop();
            gray_i2c_online = 0U;
            gray_i2c_error = NewGray_LastError();
            app_state = APP_ERROR;
        } else {
            gray_i2c_online = 1U;
            gray_i2c_error = 0U;
            publish_gray_debug(&gray);
#if HC05_TEST_STREAM_ENABLE && !APP_ENABLE_MOTORS
            /*
             * 电机安全开关关闭时也运行循迹计算，专供手推横移验证误差方向。
             * Motor_SetSidePercent在该编译模式下会强制保持全部PWM为0。
             */
            line_status = LineFollower_Update(&gray);
            line_error = LineFollower_Error();
#endif
        }

#if HC05_TEST_STREAM_ENABLE && !APP_ENABLE_MOTORS
        if ((uint32_t)(Timebase_Millis() - last_test_stream_ms) >=
            HC05_TEST_STREAM_PERIOD_MS) {
            last_test_stream_ms = Timebase_Millis();
            send_gray_test_frame();
        }
#endif

        if (gray_i2c_online == 0U) {
            continue;
        }

        Encoder_ReadAndReset(&encoder_delta);
        encoder_delta_debug = encoder_delta;

        switch (app_state) {
            case APP_WAIT_START:
                Motor_Stop();
                run_time_ms = 0U;
                if (Key_StartPressedEvent() != 0U) {
                    run_started_ms = Timebase_Millis();
                    LineFollower_Init();
                    MotorProtection_Reset(Timebase_Millis());
                    app_state = APP_RUNNING;
                }
                break;

            case APP_RUNNING:
                run_time_ms =
                    (uint32_t)(Timebase_Millis() - run_started_ms);

                /*
                 * 当前阶段忽略A点启停横线，只验证环形循迹稳定性。
                 * 从按键确认启动的时刻计时，达到30秒立即主动短刹车并锁定停止。
                 */
                if (run_time_ms >= APP_TIMED_RUN_MS) {
                    Motor_Brake();
                    app_state = APP_FINISHED;
                    break;
                }

                line_status = LineFollower_Update(&gray);
                line_error = LineFollower_Error();
                if ((line_status == LINE_ADC_ERROR) ||
                    (line_status == LINE_LOST_STOP)) {
                    Motor_Stop();
                    app_state = APP_ERROR;
                    break;
                }

                if (MotorProtection_Update(Timebase_Millis(),
                                           &encoder_delta)) {
                    Motor_Stop();
                    app_state = APP_ERROR;
                }
                break;

            case APP_FINISHED:
                Motor_Brake();
                break;

            case APP_ERROR:
            default:
                Motor_Stop();
                break;
        }
#endif
    }
}
