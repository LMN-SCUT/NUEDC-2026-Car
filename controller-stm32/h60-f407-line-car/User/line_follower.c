#include "line_follower.h"
#include "board_config.h"
#include "motor.h"
#include "timebase.h"

static const int8_t sensor_weight[GRAY_SENSOR_COUNT] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

static float previous_error;
static float current_error;
static uint8_t lost_frames;
static uint8_t line_seen;
static uint32_t boost_started_ms;

static float absolute_value(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t clamp_speed(float speed)
{
    if (speed > (float)LINE_MAX_SPEED_PERCENT) {
        return LINE_MAX_SPEED_PERCENT;
    }
    if (speed < (float)-LINE_MAX_SPEED_PERCENT) {
        return -LINE_MAX_SPEED_PERCENT;
    }
    return (int16_t)speed;
}

/* 清空历史误差和丢线状态，每次新任务开始前调用。 */
void LineFollower_Init(void)
{
    previous_error = 0.0f;
    current_error = 0.0f;
    lost_frames = 0U;
    line_seen = 0U;
    boost_started_ms = Timebase_Millis();
}

/* 供主程序和Keil Watch读取最近一次位置误差。 */
float LineFollower_Error(void)
{
    return current_error;
}

/*
 * 循迹控制主接口：
 * 1. 全白时分阶段执行短时保持、定向搜索、超时停车；
 * 2. 六路以上见黑时作为宽线状态低速直行；当前30秒测试模式不会据此停车；
 * 3. 普通纵线直接使用newgrey.c由0xDD生成的active_mask，不再做ADC阈值判断；
 * 4. S1~S8默认按车体从左到右排列，权重为-7,-5,-3,-1,+1,+3,+5,+7；
 * 5. 按见黑探头的权重平均值计算位置误差，再分段调整PD增益和基础速度；
 * 6. 最终调用Motor_SetSidePercent输出左右差速。
 */
LineFollower_Status LineFollower_Update(const GraySensor_Data *gray)
{
    int16_t weighted_sum = 0;
    uint32_t index;
    float derivative;
    float correction;
    float error_magnitude;
    float gain_scale;
    int16_t base_speed;
    int16_t left_speed;
    int16_t right_speed;

    if (gray == 0) {
        Motor_Stop();
        return LINE_ADC_ERROR;
    }

    if (gray->active_count == 0U) {
        if (lost_frames < 255U) {
            lost_frames++;
        }

        if ((line_seen == 0U) ||
            (lost_frames >= APP_LOST_STOP_FRAMES)) {
            Motor_Stop();
            return LINE_LOST_STOP;
        }

        if (lost_frames < APP_LOST_CONFIRM_FRAMES) {
            left_speed = clamp_speed(
                (float)LINE_BASE_SPEED_PERCENT +
                LINE_KP * current_error);
            right_speed = clamp_speed(
                (float)LINE_BASE_SPEED_PERCENT -
                LINE_KP * current_error);
            Motor_SetSidePercent(left_speed, right_speed);
        } else if (previous_error < 0.0f) {
            Motor_SetSidePercent(-LINE_SEARCH_SPEED_PERCENT,
                                  LINE_SEARCH_SPEED_PERCENT);
        } else {
            Motor_SetSidePercent(LINE_SEARCH_SPEED_PERCENT,
                                 -LINE_SEARCH_SPEED_PERCENT);
        }
        return LINE_TEMPORARILY_LOST;
    }

    /*
     * A点横向启停线通常会让6路以上同时见黑。宽黑不是普通位置偏差：
     * 当前30秒限时测试忽略A点启停线，因此这里只保持低速直行而不触发停车。
     * 启动时也能以低速直行离开A点，避免全黑平均值掩盖特殊标志。
     */
    if (gray->active_count >= LINE_WIDE_ACTIVE_MIN) {
        int16_t wide_speed = LINE_WIDE_SPEED_PERCENT;
        lost_frames = 0U;
        line_seen = 1U;
        current_error = 0.0f;
        previous_error = 0.0f;
        if ((uint32_t)(Timebase_Millis() - boost_started_ms) <
            LINE_START_BOOST_MS) {
            wide_speed = LINE_START_BOOST_PERCENT;
        }
        Motor_SetSidePercent(wide_speed, wide_speed);
        return LINE_WIDE_MARKER;
    }

    lost_frames = 0U;
    line_seen = 1U;
    for (index = 0U; index < GRAY_SENSOR_COUNT; index++) {
        if ((gray->active_mask & (1U << index)) != 0U) {
            weighted_sum += sensor_weight[index];
        }
    }

    current_error =
        (float)weighted_sum / (float)gray->active_count;
    error_magnitude = absolute_value(current_error);

    /*
     * 吸收旧八路查表算法“偏得越远，权重越大”的优点：
     * 中心区保持柔和；中等偏移增强20%；最外侧偏移增强50%并降速。
     * 加权平均仍覆盖全部256种位图，避免switch漏项沿用旧误差。
     */
    gain_scale = 1.0f;
    base_speed = LINE_BASE_SPEED_PERCENT;
    if (error_magnitude > LINE_MEDIUM_ERROR_LIMIT) {
        gain_scale = LINE_LARGE_GAIN_SCALE;
        base_speed = LINE_LARGE_SPEED_PERCENT;
    } else if (error_magnitude > 1.0f) {
        gain_scale = LINE_MEDIUM_GAIN_SCALE;
        base_speed = LINE_MEDIUM_SPEED_PERCENT;
    }

    /*
     * 启动后的前500 ms提高基础占空比到30%，用于克服静摩擦和电机死区。
     * 只提高基础前进量，位置PD修正仍然有效，500 ms后自动恢复分段速度。
     */
    if ((uint32_t)(Timebase_Millis() - boost_started_ms) <
        LINE_START_BOOST_MS) {
        base_speed = LINE_START_BOOST_PERCENT;
    }

    derivative = current_error - previous_error;
    correction = gain_scale *
                 (LINE_KP * current_error + LINE_KD * derivative);
    previous_error = current_error;

    left_speed = clamp_speed(
        (float)base_speed + correction);
    right_speed = clamp_speed(
        (float)base_speed - correction);
    Motor_SetSidePercent(left_speed, right_speed);
    return LINE_TRACKING;
}
