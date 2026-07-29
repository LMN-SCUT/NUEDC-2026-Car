/*
 *  ============ ti_msp_dl_config.h =============
 *  MSPM0G3507 Robot — Motor + Servo PWM + GPIO config
 *  电机: PB4=TIMA1-C0(PWM), PB6=AIN2, PB7=AIN1, PB0=STBY
 *  舵机: PB10=TIMG0-C0(PWM, 50Hz)
 */

#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_LP_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* clang-format off */

#define POWER_STARTUP_DELAY  (16)
#define CPUCLK_FREQ          (32000000)

/* ── TIMA1 PWM — 电机 (PB4) ── */
#define PWM_0_INST           TIMA1
#define PWM_0_CLK_FREQ       (32000000)   /* BUSCLK / 1 */
#define PWM_0_PERIOD         (1600)       /* 20kHz */
#define PWM_0_MAX_DUTY       (1600)

/* PB4 = IOMUX_PINCM17, func=4 = TIMA1_CCP0 */
#define GPIO_PWM_0_C0_PORT   GPIOB
#define GPIO_PWM_0_C0_PIN    DL_GPIO_PIN_4
#define GPIO_PWM_0_C0_IOMUX  (IOMUX_PINCM17)

/* ── Motor GPIO ── */
/* PB0 (STBY) = IOMUX_PINCM12 */
/* PB6 (AIN2) = IOMUX_PINCM23 */
/* PB7 (AIN1) = IOMUX_PINCM24 */

#define MOTOR_STBY_PORT      GPIOB
#define MOTOR_STBY_PIN       DL_GPIO_PIN_0
#define MOTOR_AIN2_PORT      GPIOB
#define MOTOR_AIN2_PIN       DL_GPIO_PIN_6
#define MOTOR_AIN1_PORT      GPIOB
#define MOTOR_AIN1_PIN       DL_GPIO_PIN_7

/* ── Motor control macros (TB6612 truth table) ── */
#define MOTOR_CW()    do { DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN); \
                           DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN); } while(0)
#define MOTOR_CCW()   do { DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN); \
                           DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN); } while(0)
#define MOTOR_BRAKE() do { DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN); \
                           DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN); } while(0)
#define MOTOR_STOP()  do { DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN); \
                           DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN); } while(0)
#define STBY_ON()     DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN)
#define STBY_OFF()    DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN)

/* ── TIMG0 Servo PWM (PA12, J4.8) ── */
/*
 * Timer clock: BUSCLK 32MHz / 256 (prescale=255) = 125kHz
 * Period: 125000 / 50 = 2500 → 20ms (50Hz, 舵机标准)
 * 0°:   0.5ms → 2500 * 0.5/20  ≈ 63
 * 90°:  1.5ms → 2500 * 1.5/20  ≈ 188
 * 180°: 2.5ms → 2500 * 2.5/20  ≈ 313
 */
#define SERVO_PWM_INST        TIMG0
#define SERVO_PWM_CLK_FREQ    (125000)     /* BUSCLK / 256 */
#define SERVO_PWM_PERIOD      (2500)       /* 50Hz */

/* PA12 = IOMUX_PINCM34, func=4 = TIMG0_CCP0 */
#define SERVO_PWM_PORT        GPIOA
#define SERVO_PWM_PIN         DL_GPIO_PIN_12
#define SERVO_PWM_IOMUX       (IOMUX_PINCM34)
#define SERVO_PWM_IOMUX_FUNC  IOMUX_PINCM34_PF_TIMG0_CCP0

/* Compare values for standard servo angles */
#define SERVO_ANGLE_0         63
#define SERVO_ANGLE_45        125
#define SERVO_ANGLE_90        188
#define SERVO_ANGLE_135       250
#define SERVO_ANGLE_180       313

#define servo_set(cc)         DL_TimerG_setCaptureCompareValue(SERVO_PWM_INST, (cc), DL_TIMER_CC_0_INDEX)

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_SERVO_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
