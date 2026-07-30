#include "tracking.h"
#include "board.h"
#include "gray.h"

static char tracking_mode = 'C';

/*
 * 按“循迹成功最终版”的实际运行控制移植：
 *
 * difference  = TRACK_KP * gray.error;
 * left_speed  = TRACK_BASE_SPEED + difference;
 * right_speed = TRACK_BASE_SPEED - difference;
 *
 * 1. 不使用人为强转阈值，偏差越大，负 PWM 自然越强。
 * 2. |error|>0.5 且差速不足时，强制最小差速为基础速度，
 *    克服四驱机械耦合造成的转向死区。
 * 3. difference 限制为 +/-2*base；基础速度为 80 时，
 *    最强逻辑输出为 240/-80，经电机映射后约为 100%/-40%。
 * 4. 不使用反转计时器，避免进入边缘探头前反转已经提前结束。
 *
 * 与参考工程相比，仅在电机驱动层保留 1 ms 安全换向间隔。
 */
void Tracking_Compute(float *left_speed, float *right_speed)
{
    float difference;
    float magnitude;

    if ((gray.line_found == 0U) || (gray.black_count == 0U) ||
        (gray.black_count == 8U)) {
        /*
         * 参考工程在无有效位置时只保留陀螺仪阻尼。
         * 当前灰度对照版未启用 IMU，因此位置差速置零并保持基础速度。
         */
        difference = 0.0f;
    } else {
        difference = TRACK_KP * gray.error;
        magnitude = (difference < 0.0f) ? -difference : difference;

        /* 反死区：检测到明确偏差后，至少建立 80 的左右差速。 */
        if ((gray.error > 0.5f || gray.error < -0.5f) &&
            (magnitude < TRACK_BASE_SPEED)) {
            difference =
                (gray.error < 0.0f) ? -TRACK_BASE_SPEED : TRACK_BASE_SPEED;
        }

        if (difference > TRACK_DIFF_MAX) {
            difference = TRACK_DIFF_MAX;
        }
        if (difference < -TRACK_DIFF_MAX) {
            difference = -TRACK_DIFF_MAX;
        }
    }

    *left_speed = TRACK_BASE_SPEED + difference;
    *right_speed = TRACK_BASE_SPEED - difference;

    /*
     * 不在循迹层把负数截成零；Motor_SetSides 会按正负选择 H 桥方向，
     * 并将绝对值限制在 TRACK_MAX_SPEED 对应的 100% PWM 内。
     */
    if (*right_speed < 0.0f) {
        tracking_mode = 'R';
    } else if (*left_speed < 0.0f) {
        tracking_mode = 'L';
    } else {
        tracking_mode = 'C';
    }
}

char Tracking_Mode(void)
{
    return tracking_mode;
}
