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
volatile uint8_t track_finish_armed;
volatile uint8_t track_finish_detected;
volatile uint8_t track_control_direct;

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
 * H题地图循迹遥测：
 * D/B/N是灰度原始白位图、黑线位图和见黑数量；E是位置误差x100；
 * ST是循迹状态；ARM表示已离开起点A横线；LT/RT是灰度外环目标百分比；
 * TA~TD和EA~ED分别是最近500 ms等效目标/实际累计计数；
 * PA~PD始终读取电机模块的最终PWM命令；CM=1表示大误差直接PWM，
 * 它不能单凭一帧灰度区分“地图弯道”和“直线严重歪斜”。
 */
#if !APP_MOTOR_TEST_MODE
/*
 * 纯灰度直接PWM的目标放大：
 * 第一轮原样输出21%~26%无法克服落地静摩擦，因此第二轮按1.7倍换算，
 * 并在65%处限幅。编码器反馈不会进入这个计算。
 */
static int16_t gray_target_to_direct_pwm(int16_t target_percent)
{
    int32_t scaled =
        (int32_t)target_percent * GRAY_DIRECT_PWM_SCALE_NUM;

    if (scaled >= 0) {
        scaled = (scaled + (GRAY_DIRECT_PWM_SCALE_DEN / 2)) /
                 GRAY_DIRECT_PWM_SCALE_DEN;
    } else {
        scaled = (scaled - (GRAY_DIRECT_PWM_SCALE_DEN / 2)) /
                 GRAY_DIRECT_PWM_SCALE_DEN;
    }

    if (scaled > GRAY_DIRECT_PWM_LIMIT_PERCENT) {
        scaled = GRAY_DIRECT_PWM_LIMIT_PERCENT;
    } else if (scaled < -GRAY_DIRECT_PWM_LIMIT_PERCENT) {
        scaled = -GRAY_DIRECT_PWM_LIMIT_PERCENT;
    }
    return (int16_t)scaled;
}

static void send_wait_test_frame(void)
{
    HC05_SendString("WAIT,K=");
    HC05_SendInt32((int32_t)Key_IsPressed());
    HC05_SendString(",ON=");
    HC05_SendInt32((int32_t)gray_i2c_online);
    HC05_SendString(",I2C=");
    HC05_SendInt32((int32_t)gray_i2c_error);
    HC05_SendString(",D=0x");
    HC05_SendHex8(gray_i2c_raw_mask);
    HC05_SendString(",B=0x");
    HC05_SendHex8(gray_active_mask);
    HC05_SendString(",N=");
    HC05_SendInt32((int32_t)gray_active_count);
    HC05_SendString("\r\n");
}

/* 灰度离线时也持续回传，避免安全停车后的continue形成“串口完全静默”。 */
static void send_gray_fault_frame(void)
{
    HC05_SendString("GRAY_ERROR,AS=");
    HC05_SendInt32((int32_t)app_state);
    HC05_SendString(",K=");
    HC05_SendInt32((int32_t)Key_IsPressed());
    HC05_SendString(",ON=");
    HC05_SendInt32((int32_t)gray_i2c_online);
    HC05_SendString(",I2C=");
    HC05_SendInt32((int32_t)gray_i2c_error);
    HC05_SendString(",RETRY=1\r\n");
}

static void send_track_test_frame(const Encoder_Delta *encoder_sum,
                                  const Encoder_Delta *target_sum,
                                  uint8_t finish_marker_frames)
{
    HC05_SendString("MAP,MS=");
    HC05_SendInt32((int32_t)run_time_ms);
    HC05_SendString(",AS=");
    HC05_SendInt32((int32_t)app_state);
    HC05_SendString(",ON=");
    HC05_SendInt32((int32_t)gray_i2c_online);
    HC05_SendString(",I2C=");
    HC05_SendInt32((int32_t)gray_i2c_error);
    HC05_SendString(",D=0x");
    HC05_SendHex8(gray_i2c_raw_mask);
    HC05_SendString(",B=0x");
    HC05_SendHex8(gray_active_mask);
    HC05_SendString(",N=");
    HC05_SendInt32((int32_t)gray_active_count);
    HC05_SendString(",E=");
    HC05_SendInt32((int32_t)(line_error * 100.0f));
    HC05_SendString(",ST=");
    HC05_SendInt32((int32_t)line_status);
    HC05_SendString(",ARM=");
    HC05_SendInt32((int32_t)track_finish_armed);
    HC05_SendString(",FM=");
    HC05_SendInt32((int32_t)finish_marker_frames);
    HC05_SendString(",CM=");
    HC05_SendInt32((int32_t)track_control_direct);
    HC05_SendString(",LT=");
    HC05_SendInt32((int32_t)LineFollower_LeftTargetPercent());
    HC05_SendString(",RT=");
    HC05_SendInt32((int32_t)LineFollower_RightTargetPercent());
    HC05_SendString(",TA=");
    HC05_SendInt32(target_sum->ma);
    HC05_SendString(",TB=");
    HC05_SendInt32(target_sum->mb);
    HC05_SendString(",TC=");
    HC05_SendInt32(target_sum->mc);
    HC05_SendString(",TD=");
    HC05_SendInt32(target_sum->md);
    HC05_SendString(",EA=");
    HC05_SendInt32(encoder_sum->ma);
    HC05_SendString(",EB=");
    HC05_SendInt32(encoder_sum->mb);
    HC05_SendString(",EC=");
    HC05_SendInt32(encoder_sum->mc);
    HC05_SendString(",ED=");
    HC05_SendInt32(encoder_sum->md);
    HC05_SendString(",PA=");
    HC05_SendInt32((int32_t)Motor_MACommand());
    HC05_SendString(",PB=");
    HC05_SendInt32((int32_t)Motor_MBCommand());
    HC05_SendString(",PC=");
    HC05_SendInt32((int32_t)Motor_MCCommand());
    HC05_SendString(",PD=");
    HC05_SendInt32((int32_t)Motor_MDCommand());
    HC05_SendString("\r\n");
}

/*
 * 堵转触发详情：MASK bit0~3对应MA~MD，P为最后100 ms脉冲，
 * L为连续低脉冲窗口数，C为触发检查时的逐轮PWM命令。
 */
static void send_stall_detail(void)
{
    const MotorProtection_Status *stall = MotorProtection_GetStatus();

    HC05_SendString("STALL_DETAIL,MASK=0x");
    HC05_SendHex8(stall->stalled_mask);
    HC05_SendString(",MA_P=");
    HC05_SendInt32((int32_t)stall->pulses_ma);
    HC05_SendString(",MB_P=");
    HC05_SendInt32((int32_t)stall->pulses_mb);
    HC05_SendString(",MC_P=");
    HC05_SendInt32((int32_t)stall->pulses_mc);
    HC05_SendString(",MD_P=");
    HC05_SendInt32((int32_t)stall->pulses_md);
    HC05_SendString(",MA_L=");
    HC05_SendInt32((int32_t)stall->low_windows_ma);
    HC05_SendString(",MB_L=");
    HC05_SendInt32((int32_t)stall->low_windows_mb);
    HC05_SendString(",MC_L=");
    HC05_SendInt32((int32_t)stall->low_windows_mc);
    HC05_SendString(",MD_L=");
    HC05_SendInt32((int32_t)stall->low_windows_md);
    HC05_SendString(",MA_C=");
    HC05_SendInt32((int32_t)stall->command_ma);
    HC05_SendString(",MB_C=");
    HC05_SendInt32((int32_t)stall->command_mb);
    HC05_SendString(",MC_C=");
    HC05_SendInt32((int32_t)stall->command_mc);
    HC05_SendString(",MD_C=");
    HC05_SendInt32((int32_t)stall->command_md);
    HC05_SendString("\r\n");
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
#endif
    uint32_t run_started_ms = 0U;
#if !APP_MOTOR_TEST_MODE
    GraySensor_Data gray;
    uint32_t last_track_report_ms = 0U;
    uint8_t start_marker_clear_frames = 0U;
    uint8_t finish_marker_frames = 0U;
    uint8_t gray_fault_active = 0U;
    uint32_t last_gray_retry_ms = 0U;
    Encoder_Delta track_encoder_sum = {0, 0, 0, 0};
    Encoder_Delta track_target_sum = {0, 0, 0, 0};
#endif
    Encoder_Delta encoder_delta;

    Timebase_Init();
    board_buzzer_force_off();
    HC05_Init();
#if APP_MOTOR_TEST_MODE
#if APP_FIXED_RIGHT_TURN_TEST_ENABLE
    HC05_SendString("\r\nFIXED RIGHT ARC TEST,ENCODER_PI=OFF,GROUND\r\n");
    HC05_SendString("LEFT_MB_MD=65,RIGHT_MA_MC=30,DURATION=2S,PRESS KEY\r\n");
#elif ENCODER_PI_TEST_ENABLE
    HC05_SendString("\r\nENCODER PI TEST READY,LIFT WHEELS,PRESS KEY\r\n");
    HC05_SendString("TARGET=70 COUNTS/20MS,REPORT=200MS\r\n");
#else
    HC05_SendString("\r\nENCODER TEST READY,LIFT WHEELS,PRESS KEY\r\n");
#endif
    HC05_SendString("OK bits: bit0=MA,bit1=MB,bit2=MC,bit3=MD\r\n");
#else
#if APP_TRACK_CONTROL_MODE == TRACK_CONTROL_ENCODER_PI
    HC05_SendString("\r\nH60 H-MAP TRACK,CTRL=ENCODER_PI,115200\r\n");
#elif APP_TRACK_CONTROL_MODE == TRACK_CONTROL_GRAY_PWM
    HC05_SendString("\r\nH60 H-MAP TRACK,CTRL=GRAY_PWM_X1P7,115200\r\n");
#else
    HC05_SendString("\r\nH60 H-MAP TRACK,CTRL=HYBRID_PI_PWM,115200\r\n");
#endif
    HC05_SendString("START AT A,FACE A->B,CLOCKWISE\r\n");
    HC05_SendString("MAP: T*=EQUIV_TARGET,E*=ENCODER_SUM,P*=FINAL_PWM,500MS\r\n");
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
    gray_fault_active = (gray_i2c_online == 0U) ? 1U : 0U;
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
#if APP_FIXED_RIGHT_TURN_TEST_ENABLE
                    HC05_SendString("KEY_OK,RIGHT_ARC_65_30_START,2S\r\n");
#elif ENCODER_PI_TEST_ENABLE
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
#if APP_FIXED_RIGHT_TURN_TEST_ENABLE
                    /*
                     * 编码器PI保持关闭，只记录65%/30%固定PWM下的实际轨迹
                     * 和四轮计数，不能用编码器反向修改前后轮PWM。
                     */
                    HC05_SendString("RIGHT_ARC_FINAL,TMA=");
                    HC05_SendInt32(encoder_total_ma);
                    HC05_SendString(",TMB=");
                    HC05_SendInt32(encoder_total_mb);
                    HC05_SendString(",TMC=");
                    HC05_SendInt32(encoder_total_mc);
                    HC05_SendString(",TMD=");
                    HC05_SendInt32(encoder_total_md);
                    HC05_SendString("\r\nRIGHT_ARC_TEST_DONE,STOP\r\n");
                    break;
#else
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
#endif
                }

#if APP_FIXED_RIGHT_TURN_TEST_ENABLE
                /*
                 * 固定圆弧隔离测试：左侧MB/MD为65%，右侧MA/MC为30%。
                 * 不读取灰度、不运行PID，验证底盘能否向前并形成稳定右弧。
                 */
                Motor_SetSidePercent(RIGHT_TURN_TEST_LEFT_PWM,
                                     RIGHT_TURN_TEST_RIGHT_PWM);
#elif ENCODER_PI_TEST_ENABLE
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
#elif APP_FIXED_RIGHT_TURN_TEST_ENABLE
                    RIGHT_TURN_TEST_REPORT_MS
#else
                    1000U
#endif
                    ) {
                    last_motor_test_report_ms = Timebase_Millis();
#if ENCODER_PI_TEST_ENABLE
                    {
                    const SpeedPI_Status *speed_pi_status =
                        SpeedPI_GetStatus();
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
                    }
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
            SpeedPI_Reset();
            gray_i2c_online = 0U;
            gray_i2c_error = NewGray_LastError();
            gray_fault_active = 1U;
            app_state = APP_ERROR;

            if ((uint32_t)(Timebase_Millis() -
                           last_track_report_ms) >= 1000U) {
                last_track_report_ms = Timebase_Millis();
                send_gray_fault_frame();
            }

            /*
             * 灰度模块上电可能慢于MCU，或软件I2C偶发被拉低。
             * 停车状态下每秒重新执行一次总线恢复和Ping；成功后下一帧读取
             * 会自动回到等待按键，不会在恢复瞬间自行发车。
             */
            if ((uint32_t)(Timebase_Millis() -
                           last_gray_retry_ms) >= 1000U) {
                last_gray_retry_ms = Timebase_Millis();
                (void)NewGray_Init();
            }
            continue;
        } else {
            gray_i2c_online = 1U;
            gray_i2c_error = 0U;
            publish_gray_debug(&gray);
            if (gray_fault_active != 0U) {
                gray_fault_active = 0U;
                app_state = APP_WAIT_START;
                LineFollower_Init();
                SpeedPI_Reset();
                HC05_SendString("GRAY_RECOVERED,PRESS_KEY\r\n");
            }
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

        Encoder_ReadAndReset(&encoder_delta);
        encoder_delta_debug = encoder_delta;

        switch (app_state) {
            case APP_WAIT_START:
                SpeedPI_Reset();
                run_time_ms = 0U;
                track_finish_armed = 0U;
                track_finish_detected = 0U;
                track_control_direct = 0U;
                start_marker_clear_frames = 0U;
                finish_marker_frames = 0U;
                track_encoder_sum.ma = 0;
                track_encoder_sum.mb = 0;
                track_encoder_sum.mc = 0;
                track_encoder_sum.md = 0;
                track_target_sum.ma = 0;
                track_target_sum.mb = 0;
                track_target_sum.mc = 0;
                track_target_sum.md = 0;
                if ((uint32_t)(Timebase_Millis() -
                               last_track_report_ms) >= 1000U) {
                    last_track_report_ms = Timebase_Millis();
                    send_wait_test_frame();
                }
                if (Key_StartPressedEvent() != 0U) {
                    run_started_ms = Timebase_Millis();
                    LineFollower_Init();
                    SpeedPI_Reset();
                    MotorProtection_Reset(Timebase_Millis());
                    app_state = APP_RUNNING;
                    last_track_report_ms = run_started_ms;
                    HC05_SendString("KEY_OK,MAP_RUN_START\r\n");
                }
                break;

            case APP_RUNNING:
                run_time_ms =
                    (uint32_t)(Timebase_Millis() - run_started_ms);
                track_encoder_sum.ma += encoder_delta.ma;
                track_encoder_sum.mb += encoder_delta.mb;
                track_encoder_sum.mc += encoder_delta.mc;
                track_encoder_sum.md += encoder_delta.md;

                /*
                 * 当前阶段忽略A点启停横线，只验证环形循迹稳定性。
                 * 从按键确认启动的时刻计时，达到30秒立即主动短刹车并锁定停止。
                 */
                if (run_time_ms >= APP_TIMED_RUN_MS) {
                    SpeedPI_Reset();
                    Motor_Brake();
                    app_state = APP_FINISHED;
                    HC05_SendString("TIMEOUT_30S,BRAKE\r\n");
                    break;
                }

                line_status = LineFollower_Update(&gray);
                line_error = LineFollower_Error();
                if ((line_status == LINE_ADC_ERROR) ||
                    (line_status == LINE_LOST_STOP)) {
                    SpeedPI_Reset();
                    app_state = APP_ERROR;
                    if (line_status == LINE_LOST_STOP) {
                        HC05_SendString("LINE_LOST_STOP\r\n");
                    } else {
                        HC05_SendString("GRAY_INPUT_ERROR,STOP\r\n");
                    }
                    break;
                }

                /*
                 * A点横向线状态机：
                 * 启动时车就在A横线上，必须先连续5帧离开宽黑才允许判终点；
                 * 运行至少5秒后再次连续3帧见宽黑，判定完成一圈并主动制动。
                 * 最小时间门控可过滤启动横线和近距离宽黑抖动。
                 */
                if (line_status != LINE_WIDE_MARKER) {
                    finish_marker_frames = 0U;
                    if (track_finish_armed == 0U) {
                        if (start_marker_clear_frames <
                            APP_START_MARKER_CLEAR_FRAMES) {
                            start_marker_clear_frames++;
                        }
                        if (start_marker_clear_frames >=
                            APP_START_MARKER_CLEAR_FRAMES) {
                            track_finish_armed = 1U;
                            HC05_SendString("A_MARKER_LEFT,FINISH_ARMED\r\n");
                        }
                    }
                } else if ((track_finish_armed != 0U) &&
                           (run_time_ms >= APP_FINISH_MIN_MS)) {
                    if (finish_marker_frames <
                        APP_FINISH_MARKER_FRAMES) {
                        finish_marker_frames++;
                    }
                    if (finish_marker_frames >=
                        APP_FINISH_MARKER_FRAMES) {
                        track_finish_detected = 1U;
                        SpeedPI_Reset();
                        Motor_Brake();
                        app_state = APP_FINISHED;
                        HC05_SendString("A_FINISH_DETECTED,MS=");
                        HC05_SendInt32((int32_t)run_time_ms);
                        HC05_SendString(",BRAKE\r\n");
                        break;
                    }
                }

                /*
                 * 三种控制模式共用灰度、编码器遥测和堵转保护：
                 * 1. ENCODER_PI：全程由四路速度PI写PWM；
                 * 2. GRAY_PWM：灰度左右百分比直接写PWM；
                 * 3. HYBRID：小误差用PI，大误差/丢线直接给外侧60%、内侧15%。
                 * 混合模式切换时清空PI积分，防止回到直线后沿用弯道历史补偿。
                 */
#if APP_TRACK_CONTROL_MODE == TRACK_CONTROL_ENCODER_PI
                track_control_direct = 0U;
                SpeedPI_SetSidePercentTargets(
                    LineFollower_LeftTargetPercent(),
                    LineFollower_RightTargetPercent());
                SpeedPI_Update(&encoder_delta);
                {
                    const SpeedPI_Status *speed = SpeedPI_GetStatus();
                    track_target_sum.ma += speed->target_ma;
                    track_target_sum.mb += speed->target_mb;
                    track_target_sum.mc += speed->target_mc;
                    track_target_sum.md += speed->target_md;
                }
#elif APP_TRACK_CONTROL_MODE == TRACK_CONTROL_GRAY_PWM
                {
                    int16_t direct_left_pwm =
                        gray_target_to_direct_pwm(
                            LineFollower_LeftTargetPercent());
                    int16_t direct_right_pwm =
                        gray_target_to_direct_pwm(
                            LineFollower_RightTargetPercent());

                track_control_direct = 1U;
                    Motor_SetSidePercent(direct_left_pwm,
                                         direct_right_pwm);
                /*
                     * TA~TD按最终PWM换算为等效计数，便于和EA~ED观察趋势；
                     * 它不是闭环速度目标，编码器不会反向修改PA~PD。
                 */
                track_target_sum.ma +=
                        (int32_t)direct_right_pwm *
                    (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                track_target_sum.mb +=
                        (int32_t)direct_left_pwm *
                    (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                track_target_sum.mc +=
                        (int32_t)direct_right_pwm *
                    (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                track_target_sum.md +=
                        (int32_t)direct_left_pwm *
                    (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                }
#else
                if ((line_status == LINE_TEMPORARILY_LOST) ||
                    (line_error >= HYBRID_DIRECT_ERROR_THRESHOLD) ||
                    (line_error <= -HYBRID_DIRECT_ERROR_THRESHOLD)) {
                    int16_t direct_left_pwm;
                    int16_t direct_right_pwm;

                    if (track_control_direct == 0U) {
                        SpeedPI_Reset();
                    }
                    track_control_direct = 1U;

                    /*
                     * LT>RT说明需要右转，左轮为外侧；反之右轮为外侧。
                     * 丢线时LineFollower仍保留最后一次搜索方向，因此同样适用。
                     */
                    if (LineFollower_LeftTargetPercent() >=
                        LineFollower_RightTargetPercent()) {
                        direct_left_pwm = HYBRID_CURVE_OUTER_PWM;
                        direct_right_pwm = HYBRID_CURVE_INNER_PWM;
                    } else {
                        direct_left_pwm = HYBRID_CURVE_INNER_PWM;
                        direct_right_pwm = HYBRID_CURVE_OUTER_PWM;
                    }
                    Motor_SetSidePercent(direct_left_pwm,
                                         direct_right_pwm);

                    track_target_sum.ma +=
                        (int32_t)direct_right_pwm *
                        (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                    track_target_sum.mb +=
                        (int32_t)direct_left_pwm *
                        (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                    track_target_sum.mc +=
                        (int32_t)direct_right_pwm *
                        (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                    track_target_sum.md +=
                        (int32_t)direct_left_pwm *
                        (int32_t)SPEED_PI_COUNTS_PER_PERCENT;
                } else {
                    if (track_control_direct != 0U) {
                        SpeedPI_Reset();
                    }
                    track_control_direct = 0U;
                    SpeedPI_SetSidePercentTargets(
                        LineFollower_LeftTargetPercent(),
                        LineFollower_RightTargetPercent());
                    SpeedPI_Update(&encoder_delta);
                    {
                        const SpeedPI_Status *speed = SpeedPI_GetStatus();
                        track_target_sum.ma += speed->target_ma;
                        track_target_sum.mb += speed->target_mb;
                        track_target_sum.mc += speed->target_mc;
                        track_target_sum.md += speed->target_md;
                    }
                }
#endif

                if (MotorProtection_Update(Timebase_Millis(),
                                           &encoder_delta)) {
                    send_stall_detail();
                    SpeedPI_Reset();
                    app_state = APP_ERROR;
                    HC05_SendString("STALL_OR_ENCODER_ERROR,STOP\r\n");
                    break;
                }

                if ((uint32_t)(Timebase_Millis() -
                               last_track_report_ms) >=
                    APP_TRACK_REPORT_MS) {
                    last_track_report_ms = Timebase_Millis();
                    send_track_test_frame(&track_encoder_sum,
                                          &track_target_sum,
                                          finish_marker_frames);
                    track_encoder_sum.ma = 0;
                    track_encoder_sum.mb = 0;
                    track_encoder_sum.mc = 0;
                    track_encoder_sum.md = 0;
                    track_target_sum.ma = 0;
                    track_target_sum.mb = 0;
                    track_target_sum.mc = 0;
                    track_target_sum.md = 0;
                }
                break;

            case APP_FINISHED:
                Motor_Brake();
                if ((uint32_t)(Timebase_Millis() -
                               last_track_report_ms) >= 1000U) {
                    last_track_report_ms = Timebase_Millis();
                    HC05_SendString("STATE,FINISHED,MS=");
                    HC05_SendInt32((int32_t)run_time_ms);
                    HC05_SendString("\r\n");
                }
                break;

            case APP_ERROR:
            default:
                SpeedPI_Reset();
                if ((uint32_t)(Timebase_Millis() -
                               last_track_report_ms) >= 1000U) {
                    last_track_report_ms = Timebase_Millis();
                    HC05_SendString("STATE,ERROR,ON=");
                    HC05_SendInt32((int32_t)gray_i2c_online);
                    HC05_SendString(",I2C=");
                    HC05_SendInt32((int32_t)gray_i2c_error);
                    HC05_SendString("\r\n");
                    if (MotorProtection_GetStatus()->stalled_mask != 0U) {
                        send_stall_detail();
                    }
                }
                break;
        }
#endif
    }
}
