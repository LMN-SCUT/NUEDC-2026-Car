/*
 *  ============ ti_msp_dl_config.c =============
 *  MSPM0G3507 Robot — System + Motor PWM + Servo PWM init
 */

#include "ti_msp_dl_config.h"

/* ── TIMA1 clock: BUSCLK/1, 32MHz ── */
static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U
};

/* ── TIMA1 PWM: edge-aligned, 20kHz, 4 CC registers (TIMA is advanced) ── */
static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period            = PWM_0_PERIOD,
    .isTimerWithFourCC = true,
    .startTimer        = DL_TIMER_STOP,
};

/* ── TIMG0 clock: BUSCLK/1, prescale=255 → 125kHz ── */
static const DL_TimerG_ClockConfig gSERVO_ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 255U
};

/*
 * TIMG0 PWM config: edge-aligned, period=2500, 50Hz
 * isTimerWithFourCC=false → initFourCCPWMMode 跳过 CC2/CC3，只调 initTwoCCPWMMode
 * TIMG 只有 2 个 CC，必须设 false
 */
static const DL_TimerG_PWMConfig gSERVO_Config = {
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,  /* 向上计数，自然输出正向脉冲 */
    .period            = SERVO_PWM_PERIOD,
    .isTimerWithFourCC = false,
    .startTimer        = DL_TIMER_STOP,
};

/* ======== SYSCFG_DL_init ======== */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_0_init();
    SYSCFG_DL_SERVO_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(SERVO_PWM_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(SERVO_PWM_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    /* PB4 → TIMA1_CCP0 (motor PWM output) */
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_0_C0_IOMUX, IOMUX_PINCM17_PF_TIMA1_CCP0);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);

    /* PB10 → TIMG0_CCP0 (servo PWM output) */
    DL_GPIO_initPeripheralOutputFunction(
        SERVO_PWM_IOMUX, SERVO_PWM_IOMUX_FUNC);
    DL_GPIO_enableOutput(SERVO_PWM_PORT, SERVO_PWM_PIN);

    /* PB0 → GPIO output (STBY) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM12);
    DL_GPIO_enableOutput(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
    /* PB6 → GPIO output (AIN2) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM23);
    DL_GPIO_enableOutput(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    /* PB7 → GPIO output (AIN1) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM24);
    DL_GPIO_enableOutput(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);

    /* PB26 → GPIO output (RGB LED RED — 诊断用) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM57);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_26);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_26);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
}

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void)
{
    DL_TimerA_setClockConfig(
        PWM_0_INST, (DL_TimerA_ClockConfig *)&gPWM_0ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_0_INST, (DL_TimerA_PWMConfig *)&gPWM_0Config);

    DL_TimerA_setCounterControl(
        PWM_0_INST,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    /* Channel 0 output: init low, no invert, source = function value */
    DL_TimerA_setCaptureCompareOutCtl(
        PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(
        PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    /* Start with 0% duty */
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_enableClock(PWM_0_INST);
    DL_TimerA_setCCPDirection(PWM_0_INST, DL_TIMER_CC0_OUTPUT);
}

SYSCONFIG_WEAK void SYSCFG_DL_SERVO_init(void)
{
    /*
     * TIMG0 edge-aligned UP PWM, period=2500, 50Hz
     * 完全照搬 LCKFB 社区例程的 init 模式：
     *   initPWMMode → setCC=0 → setOCTL → setUpdateMethod → enableClock → setCCPD
     *   startCounter 在 main() 里调！
     *
     * EDGE_ALIGN_UP: 计数器从 0 向上数到 LOAD
     *   0→HIGH, CC→LOW, LOAD→归零→HIGH → 自然产生正向脉冲
     *   不需要 INIT_VAL_HIGH hack
     */

    DL_TimerG_setClockConfig(
        SERVO_PWM_INST, (DL_TimerG_ClockConfig *)&gSERVO_ClockConfig);

    DL_TimerG_initPWMMode(
        SERVO_PWM_INST, (DL_TimerG_PWMConfig *)&gSERVO_Config);

    /* 初始占空比=0, main() 里再设角度 */
    DL_TimerG_setCaptureCompareValue(
        SERVO_PWM_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(
        SERVO_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(
        SERVO_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_enableClock(SERVO_PWM_INST);
    DL_TimerG_setCCPDirection(SERVO_PWM_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
}

