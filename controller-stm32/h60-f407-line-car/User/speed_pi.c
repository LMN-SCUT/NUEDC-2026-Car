#include "speed_pi.h"
#include "board_config.h"
#include "motor.h"

typedef struct {
    float integral;
} WheelPI;

static WheelPI pi_ma;
static WheelPI pi_mb;
static WheelPI pi_mc;
static WheelPI pi_md;
static SpeedPI_Status speed_status;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

/*
 * 单轮速度PI：
 * 1. 前馈项根据目标计数直接给出接近工作点的PWM，减少纯积分慢慢爬升。
 * 2. P项修正当前20 ms速度误差。
 * 3. I项补偿电池电压、地面阻力和四个电机的长期差异。
 * 4. 积分及最终PWM均限幅，避免堵转时无限累积。
 */
static int16_t update_one_wheel(WheelPI *pi, int32_t target,
                                int32_t measured)
{
    float error;
    float feedforward;
    float output;

    if (target == 0) {
        pi->integral = 0.0f;
        return 0;
    }

    error = (float)(target - measured);
    pi->integral += SPEED_PI_KI * error;
    /*
     * 积分只补偿当前目标方向的负载，不允许积累出相反方向的驱动力。
     * 正目标积分范围0~上限，负目标积分范围-上限~0。
     */
    if (target > 0) {
        pi->integral = clamp_float(
            pi->integral, 0.0f, SPEED_PI_INTEGRAL_LIMIT_PERCENT);
    } else {
        pi->integral = clamp_float(
            pi->integral, -SPEED_PI_INTEGRAL_LIMIT_PERCENT, 0.0f);
    }

    feedforward = SPEED_PI_FEEDFORWARD_PERCENT_PER_COUNT *
                  (float)target;
    output = feedforward + SPEED_PI_KP * error + pi->integral;
    output = clamp_float(output, -SPEED_PI_PWM_LIMIT_PERCENT,
                         SPEED_PI_PWM_LIMIT_PERCENT);

    /*
     * 目标为正时只允许正转或滑行，目标为负时只允许反转或滑行。
     * 这样内轮突然降速时PI不会用反向PWM主动制动，避免四驱底盘在弯道
     * 形成“左轮前进、右轮反转”的高阻力原地差速状态。
     */
    if (((target > 0) && (output < 0.0f)) ||
        ((target < 0) && (output > 0.0f))) {
        output = 0.0f;
    }

    if (output >= 0.0f) {
        return (int16_t)(output + 0.5f);
    }
    return (int16_t)(output - 0.5f);
}

/* 初始化只建立软件状态，不配置GPIO；PWM硬件仍由Motor_Init负责。 */
void SpeedPI_Init(void)
{
    SpeedPI_Reset();
}

/* 停车时同时清空积分，保证下一次启动不会带着旧PWM补偿。 */
void SpeedPI_Reset(void)
{
    pi_ma.integral = 0.0f;
    pi_mb.integral = 0.0f;
    pi_mc.integral = 0.0f;
    pi_md.integral = 0.0f;
    speed_status.target_ma = 0;
    speed_status.target_mb = 0;
    speed_status.target_mc = 0;
    speed_status.target_md = 0;
    speed_status.pwm_ma = 0;
    speed_status.pwm_mb = 0;
    speed_status.pwm_mc = 0;
    speed_status.pwm_md = 0;
    Motor_Stop();
}

/* 设置目标不会立即写PWM；下一次SpeedPI_Update才使用最新编码器反馈。 */
void SpeedPI_SetTargets(int32_t ma, int32_t mb, int32_t mc, int32_t md)
{
    speed_status.target_ma = ma;
    speed_status.target_mb = mb;
    speed_status.target_mc = mc;
    speed_status.target_md = md;
}

/*
 * 将循迹外环的抽象百分比转换成每20 ms编码器计数。
 * 当前换算系数来自40%落地约77~88计数的实测，只作为首轮低速标定值。
 */
void SpeedPI_SetSidePercentTargets(int16_t left_percent,
                                   int16_t right_percent)
{
    int32_t left_target =
        (int32_t)((float)left_percent * SPEED_PI_COUNTS_PER_PERCENT);
    int32_t right_target =
        (int32_t)((float)right_percent * SPEED_PI_COUNTS_PER_PERCENT);

    /*
     * H60板背面接口：左列为MB/MD，右列为MA/MC。
     * 参数顺序为MA、MB、MC、MD，因此右、左、右、左交错写入。
     */
    SpeedPI_SetTargets(right_target, left_target,
                       right_target, left_target);
}
/* 四个PI分别计算，最终只有Motor_SetWheelPercent负责写入硬件PWM。 */
void SpeedPI_Update(const Encoder_Delta *measured)
{
    if (measured == 0) {
        return;
    }

    speed_status.pwm_ma =
        update_one_wheel(&pi_ma, speed_status.target_ma, measured->ma);
    speed_status.pwm_mb =
        update_one_wheel(&pi_mb, speed_status.target_mb, measured->mb);
    speed_status.pwm_mc =
        update_one_wheel(&pi_mc, speed_status.target_mc, measured->mc);
    speed_status.pwm_md =
        update_one_wheel(&pi_md, speed_status.target_md, measured->md);

    Motor_SetWheelPercent(speed_status.pwm_ma, speed_status.pwm_mb,
                          speed_status.pwm_mc, speed_status.pwm_md);
}

const SpeedPI_Status *SpeedPI_GetStatus(void)
{
    return &speed_status;
}
