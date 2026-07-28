#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define BRIDGE_BUFFER_SIZE 256U
#define ACTIVITY_HOLD_MS 40U
#define HEARTBEAT_PERIOD_MS 500U

typedef struct {
    uint8_t data[BRIDGE_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} byte_ring_t;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;

static byte_ring_t g_hc05_to_pc;
static byte_ring_t g_pc_to_hc05;
static uint8_t g_hc05_rx_byte;
static uint8_t g_pc_rx_byte;
static uint32_t g_activity_until_ms;

/* Inspect these counters in Keil Watch if a terminal is unavailable. */
volatile uint32_t g_hc05_to_pc_bytes;
volatile uint32_t g_pc_to_hc05_bytes;
volatile uint32_t g_hc05_rx_overflow;
volatile uint32_t g_pc_rx_overflow;
volatile uint32_t g_hc05_uart_errors;
volatile uint32_t g_pc_uart_errors;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);

static bool ring_push(byte_ring_t *ring, uint8_t byte)
{
    uint16_t head = ring->head;
    uint16_t next = (uint16_t)((head + 1U) % BRIDGE_BUFFER_SIZE);

    if (next == ring->tail) {
        return false;
    }
    ring->data[head] = byte;
    ring->head = next;
    return true;
}

static bool ring_pop(byte_ring_t *ring, uint8_t *byte)
{
    uint16_t tail = ring->tail;

    if (tail == ring->head) {
        return false;
    }
    *byte = ring->data[tail];
    ring->tail = (uint16_t)((tail + 1U) % BRIDGE_BUFFER_SIZE);
    return true;
}

static void bridge_process(void)
{
    uint8_t byte;

    while (ring_pop(&g_hc05_to_pc, &byte)) {
        if (HAL_UART_Transmit(&huart1, &byte, 1U, 10U) == HAL_OK) {
            g_hc05_to_pc_bytes++;
            g_activity_until_ms = HAL_GetTick() + ACTIVITY_HOLD_MS;
        } else {
            g_pc_uart_errors++;
            break;
        }
    }

    while (ring_pop(&g_pc_to_hc05, &byte)) {
        if (HAL_UART_Transmit(&huart4, &byte, 1U, 10U) == HAL_OK) {
            g_pc_to_hc05_bytes++;
            g_activity_until_ms = HAL_GetTick() + ACTIVITY_HOLD_MS;
        } else {
            g_hc05_uart_errors++;
            break;
        }
    }
}

static void update_leds(uint32_t now_ms)
{
    GPIO_PinState heartbeat =
        ((now_ms / HEARTBEAT_PERIOD_MS) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(BRIDGE_HEARTBEAT_LED_GPIO_Port,
        BRIDGE_HEARTBEAT_LED_Pin, heartbeat);
    HAL_GPIO_WritePin(BRIDGE_ACTIVITY_LED_GPIO_Port, BRIDGE_ACTIVITY_LED_Pin,
        ((int32_t)(g_activity_until_ms - now_ms) > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_UART4_Init();

    HAL_GPIO_WritePin(BRIDGE_HEARTBEAT_LED_GPIO_Port,
        BRIDGE_HEARTBEAT_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BRIDGE_ACTIVITY_LED_GPIO_Port,
        BRIDGE_ACTIVITY_LED_Pin, GPIO_PIN_SET);

    if (HAL_UART_Receive_IT(&huart4, &g_hc05_rx_byte, 1U) != HAL_OK ||
        HAL_UART_Receive_IT(&huart1, &g_pc_rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        bridge_process();
        update_leds(HAL_GetTick());
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

static void uart_init(UART_HandleTypeDef *uart, USART_TypeDef *instance)
{
    uart->Instance = instance;
    uart->Init.BaudRate = 115200;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    uart_init(&huart1, USART1);
}

static void MX_UART4_Init(void)
{
    uart_init(&huart4, UART4);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART4) {
        if (!ring_push(&g_hc05_to_pc, g_hc05_rx_byte)) {
            g_hc05_rx_overflow++;
        }
        (void)HAL_UART_Receive_IT(&huart4, &g_hc05_rx_byte, 1U);
    } else if (uart->Instance == USART1) {
        if (!ring_push(&g_pc_to_hc05, g_pc_rx_byte)) {
            g_pc_rx_overflow++;
        }
        (void)HAL_UART_Receive_IT(&huart1, &g_pc_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART4) {
        g_hc05_uart_errors++;
        (void)HAL_UART_Receive_IT(&huart4, &g_hc05_rx_byte, 1U);
    } else if (uart->Instance == USART1) {
        g_pc_uart_errors++;
        (void)HAL_UART_Receive_IT(&huart1, &g_pc_rx_byte, 1U);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(BRIDGE_ACTIVITY_LED_GPIO_Port,
            BRIDGE_ACTIVITY_LED_Pin);
        for (volatile uint32_t delay = 0U; delay < 200000U; delay++) {
        }
    }
}
