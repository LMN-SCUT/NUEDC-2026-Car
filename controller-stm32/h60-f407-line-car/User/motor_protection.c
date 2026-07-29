#include "motor_protection.h"
#include "board_config.h"
#include "motor.h"

typedef struct {
    uint32_t pulses;
    uint8_t low_pulse_windows;
} Wheel_Protection;

static Wheel_Protection ma_protection;
static Wheel_Protection mb_protection;
static Wheel_Protection mc_protection;
static Wheel_Protection md_protection;
static uint32_t last_check_ms;
static uint32_t grace_started_ms;
static bool stall_latched;
static MotorProtection_Status protection_status;

/* 取编码器增量绝对值；堵转判断只关心是否运动，不依赖方向符号。 */
static uint32_t pulse_magnitude(int32_t value)
{
    if (value < 0) {
        return (uint32_t)(-(value + 1)) + 1U;
    }
    return (uint32_t)value;
}

static int16_t command_magnitude(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

/* 清空单个车轮的窗口脉冲和连续低脉冲次数。 */
static void reset_wheel(Wheel_Protection *wheel)
{
    wheel->pulses = 0U;
    wheel->low_pulse_windows = 0U;
}

/*
 * 完成一个车轮的100 ms窗口判断。
 * should_move=false时清除历史，防止停车阶段被误判为堵转。
 */
static bool check_wheel(Wheel_Protection *wheel, bool should_move)
{
    if (!should_move) {
        reset_wheel(wheel);
        return false;
    }

    if (wheel->pulses < STALL_MIN_PULSES_PER_WINDOW) {
        if (wheel->low_pulse_windows < 255U) {
            wheel->low_pulse_windows++;
        }
    } else {
        wheel->low_pulse_windows = 0U;
    }
    wheel->pulses = 0U;

    return wheel->low_pulse_windows >= STALL_CONFIRM_WINDOWS;
}

/*
 * 初始化堵转保护。stall_latched一旦置位，只能通过本接口或Reset显式清除；
 * 主程序触发APP_ERROR后不会自动调用Reset，因此故障不会自行恢复。
 */
void MotorProtection_Init(uint32_t now_ms)
{
    MotorProtection_Reset(now_ms);
}

/* 新任务启动前清除全部窗口累计和锁存故障。 */
void MotorProtection_Reset(uint32_t now_ms)
{
    reset_wheel(&ma_protection);
    reset_wheel(&mb_protection);
    reset_wheel(&mc_protection);
    reset_wheel(&md_protection);
    last_check_ms = now_ms;
    grace_started_ms = now_ms;
    stall_latched = false;
    protection_status.pulses_ma = 0U;
    protection_status.pulses_mb = 0U;
    protection_status.pulses_mc = 0U;
    protection_status.pulses_md = 0U;
    protection_status.low_windows_ma = 0U;
    protection_status.low_windows_mb = 0U;
    protection_status.low_windows_mc = 0U;
    protection_status.low_windows_md = 0U;
    protection_status.command_ma = 0;
    protection_status.command_mb = 0;
    protection_status.command_mc = 0;
    protection_status.command_md = 0;
    protection_status.stalled_mask = 0U;
}

/*
 * 每个控制周期调用。先累计四路脉冲，到达100 ms窗口后再统一判断，
 * 避免用单个20 ms周期的少量脉冲直接误判。
 */
bool MotorProtection_Update(uint32_t now_ms, const Encoder_Delta *delta)
{
#if STALL_PROTECTION_ENABLE
    bool ma_should_move;
    bool mb_should_move;
    bool mc_should_move;
    bool md_should_move;
    bool ma_stalled;
    bool mb_stalled;
    bool mc_stalled;
    bool md_stalled;
    bool stalled;

    if (stall_latched) {
        Motor_Stop();
        return true;
    }
    if (delta == 0) {
        Motor_Stop();
        stall_latched = true;
        protection_status.stalled_mask = 0x0FU;
        return true;
    }

    /*
     * 新任务启动后的前1秒为起转宽限期：不累计低脉冲窗口，也不判堵转。
     * 宽限期只用于跨过静摩擦和加速阶段，结束后从全新100 ms窗口开始检测。
     */
    if ((uint32_t)(now_ms - grace_started_ms) <
        STALL_STARTUP_GRACE_MS) {
        reset_wheel(&ma_protection);
        reset_wheel(&mb_protection);
        reset_wheel(&mc_protection);
        reset_wheel(&md_protection);
        protection_status.pulses_ma = 0U;
        protection_status.pulses_mb = 0U;
        protection_status.pulses_mc = 0U;
        protection_status.pulses_md = 0U;
        protection_status.low_windows_ma = 0U;
        protection_status.low_windows_mb = 0U;
        protection_status.low_windows_mc = 0U;
        protection_status.low_windows_md = 0U;
        protection_status.command_ma = Motor_MACommand();
        protection_status.command_mb = Motor_MBCommand();
        protection_status.command_mc = Motor_MCCommand();
        protection_status.command_md = Motor_MDCommand();
        protection_status.stalled_mask = 0U;
        last_check_ms = now_ms;
        return false;
    }

    ma_protection.pulses += pulse_magnitude(delta->ma);
    mb_protection.pulses += pulse_magnitude(delta->mb);
    mc_protection.pulses += pulse_magnitude(delta->mc);
    md_protection.pulses += pulse_magnitude(delta->md);

    if ((uint32_t)(now_ms - last_check_ms) < STALL_CHECK_PERIOD_MS) {
        return false;
    }
    last_check_ms = now_ms;

    protection_status.pulses_ma = ma_protection.pulses;
    protection_status.pulses_mb = mb_protection.pulses;
    protection_status.pulses_mc = mc_protection.pulses;
    protection_status.pulses_md = md_protection.pulses;
    protection_status.command_ma = Motor_MACommand();
    protection_status.command_mb = Motor_MBCommand();
    protection_status.command_mc = Motor_MCCommand();
    protection_status.command_md = Motor_MDCommand();

    ma_should_move = command_magnitude(protection_status.command_ma) >=
                     STALL_COMMAND_MIN_PERCENT;
    mb_should_move = command_magnitude(protection_status.command_mb) >=
                     STALL_COMMAND_MIN_PERCENT;
    mc_should_move = command_magnitude(protection_status.command_mc) >=
                     STALL_COMMAND_MIN_PERCENT;
    md_should_move = command_magnitude(protection_status.command_md) >=
                     STALL_COMMAND_MIN_PERCENT;

    ma_stalled = check_wheel(&ma_protection, ma_should_move);
    mb_stalled = check_wheel(&mb_protection, mb_should_move);
    mc_stalled = check_wheel(&mc_protection, mc_should_move);
    md_stalled = check_wheel(&md_protection, md_should_move);

    protection_status.low_windows_ma = ma_protection.low_pulse_windows;
    protection_status.low_windows_mb = mb_protection.low_pulse_windows;
    protection_status.low_windows_mc = mc_protection.low_pulse_windows;
    protection_status.low_windows_md = md_protection.low_pulse_windows;
    protection_status.stalled_mask =
        (ma_stalled ? 0x01U : 0U) |
        (mb_stalled ? 0x02U : 0U) |
        (mc_stalled ? 0x04U : 0U) |
        (md_stalled ? 0x08U : 0U);
    stalled = ma_stalled || mb_stalled || mc_stalled || md_stalled;

    if (stalled) {
        Motor_Stop();
        stall_latched = true;
        return true;
    }
    return false;
#else
    (void)now_ms;
    (void)delta;
    return false;
#endif
}

const MotorProtection_Status *MotorProtection_GetStatus(void)
{
    return &protection_status;
}
