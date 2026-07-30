#include "board.h"
#include "gray.h"
#include "key.h"
#include "motor.h"
#include "serial.h"
#include "timebase.h"
#include "tracking.h"

int main(void)
{
    uint32_t run_start;
    uint32_t last_report;
    uint8_t left_start_marker;
    uint8_t finish_samples;
    float left_speed;
    float right_speed;

    Board_Init();
    Serial_Init();
    Key_Init();
    Motor_Init();

    /*
     * 成功工程在 Gray_Init 前至少等待 1 s，使灰度模块电源和内部 MCU
     * 完成启动。当前工程同样保留这段稳定时间，再进行 0xAA/0x66 握手。
     */
    DelayMs(1000U);
    Gray_Init();

    Serial_Printf("\r\nH60 H-TRACK SPL, GRAY_REF_I2C, NO_GYRO\r\n");
    Serial_Printf("START=A, FACE A->B, CLOCKWISE; PRESS K\r\n");
    Serial_Printf("GRAY=%u, PC2=SDA PC3=SCL\r\n",
                  gray.communication_ok);

    while (1) {
        uint8_t gray_ok = Gray_Read();
        if ((Key_PressedEvent() != 0U) && (gray_ok != 0U) &&
            (gray.communication_ok != 0U)) {
            break;
        }
        if ((Millis() % 500U) < 5U) {
            Serial_Printf("WAIT,M=0x%02X,N=%u,E=%+.1f,I2C=%u,GE=%lu,GS=%u\r\n",
                          gray.mask, gray.black_count, gray.error,
                          gray.communication_ok, gray.error_count,
                          gray.error_stage);
            DelayMs(5U);
        }
    }

    run_start = Millis();
    last_report = run_start;
    left_start_marker = 0U;
    finish_samples = 0U;
    Serial_Printf("RUN_START,A_MASK=0x%02X,A_N=%u\r\n",
                  gray.mask, gray.black_count);

    while (1) {
        uint32_t elapsed = Millis() - run_start;

        /*
         * Gray_Read 内部已经执行 3 次事务与总线恢复。若仍失败，禁止使用
         * 最后一帧旧误差继续驱动车辆；立即停车并锁定，复位后重新发车。
         */
        if (Gray_Read() == 0U) {
            Motor_StopAll();
            Serial_Printf("GRAY_FAIL_STOP,MS=%lu,GE=%lu,GS=%u\r\n",
                          elapsed, gray.error_count, gray.error_stage);
            break;
        }
        Tracking_Compute(&left_speed, &right_speed);
        Motor_SetSides(left_speed, right_speed);

        /* 必须先离开 A 横线，防止按键后立刻把起点误判为终点。 */
        if ((elapsed > 500U) && (gray.black_count <= 2U)) {
            left_start_marker = 1U;
        }

        if ((left_start_marker != 0U) && (elapsed >= FINISH_MIN_TIME_MS) &&
            (gray.black_count >= FINISH_BLACK_MIN)) {
            if (finish_samples < 255U) finish_samples++;
        } else {
            finish_samples = 0U;
        }

        if (finish_samples >= FINISH_STABLE_SAMPLES) {
            Motor_StopAll();
            Serial_Printf("FINISH_A,MS=%lu,M=0x%02X,N=%u\r\n",
                          elapsed, gray.mask, gray.black_count);
            break;
        }

        if (elapsed >= RUN_TIMEOUT_MS) {
            Motor_StopAll();
            Serial_Printf("TIMEOUT_STOP,MS=%lu,M=0x%02X,N=%u\r\n",
                          elapsed, gray.mask, gray.black_count);
            break;
        }

        if ((uint32_t)(Millis() - last_report) >= 200U) {
            Serial_Printf("RUN,MS=%lu,M=0x%02X,N=%u,E=%+.1f,TM=%c,"
                          "L=%.0f,R=%.0f,PL=%u,PR=%u,I2C=%u,"
                          "GE=%lu,GS=%u\r\n",
                          elapsed, gray.mask, gray.black_count, gray.error,
                          Tracking_Mode(),
                          left_speed, right_speed,
                          Motor_LeftDuty(), Motor_RightDuty(),
                          gray.communication_ok, gray.error_count,
                          gray.error_stage);
            last_report = Millis();
        }
        DelayMs(5U);
    }

    while (1) {
        Motor_StopAll();
    }
}
