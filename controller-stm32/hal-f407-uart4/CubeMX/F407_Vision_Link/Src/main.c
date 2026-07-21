/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vision_link_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static vl_hal_link_t g_vision_link;
static vl_observation_t g_latest_observation;
static vl_ack_t g_latest_ack;
static uint8_t g_command_seq;
static uint8_t g_pending_command_seq;
static uint8_t g_command_pending;
static uint8_t g_command_retry_count;
static uint8_t g_next_mode;
static uint32_t g_last_command_ms;
static uint32_t g_ack_led_until_ms;
static uint32_t g_ack_success_count;
static uint32_t g_ack_timeout_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  /* Power-on lamp test: both onboard user LEDs are active-low. */
  HAL_GPIO_WritePin(VISION_LINK_LED_GPIO_Port, VISION_LINK_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VISION_ACTIVITY_LED_GPIO_Port, VISION_ACTIVITY_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(150u);
  HAL_GPIO_WritePin(VISION_LINK_LED_GPIO_Port, VISION_LINK_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VISION_ACTIVITY_LED_GPIO_Port, VISION_ACTIVITY_LED_Pin, GPIO_PIN_SET);

  vl_hal_init(&g_vision_link, &huart4);
  if (vl_hal_start_rx(&g_vision_link) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();
    vl_command_t command;

    vl_hal_process(&g_vision_link, now_ms);
    if (vl_hal_take_observation(&g_vision_link, &g_latest_observation))
    {
      /* Observation is retained for the application/control layer. */
    }

    if (vl_hal_take_ack(&g_vision_link, &g_latest_ack))
    {
      if (g_command_pending != 0u
          && g_latest_ack.request_type == VL_TYPE_COMMAND
          && g_latest_ack.request_seq == g_pending_command_seq
          && g_latest_ack.status == 0u)
      {
        g_command_pending = 0u;
        g_ack_success_count++;
        g_next_mode ^= 1u;
        g_ack_led_until_ms = now_ms + 150u;
      }
    }

    if (g_command_pending == 0u && (uint32_t)(now_ms - g_last_command_ms) >= 2000u)
    {
      command.command_id = 0x01u; /* SET_MODE */
      command.mode = g_next_mode;
      command.arg0 = 0;
      command.arg1 = 0;
      command.arg2 = 0;
      g_pending_command_seq = g_command_seq++;
      if (vl_hal_send_command(&g_vision_link, g_pending_command_seq, &command) == HAL_OK)
      {
        g_command_pending = 1u;
        g_command_retry_count = 0u;
        g_last_command_ms = now_ms;
      }
    }
    else if (g_command_pending != 0u && (uint32_t)(now_ms - g_last_command_ms) >= 100u)
    {
      if (g_command_retry_count < 2u)
      {
        command.command_id = 0x01u;
        command.mode = g_next_mode;
        command.arg0 = 0;
        command.arg1 = 0;
        command.arg2 = 0;
        (void)vl_hal_send_command(&g_vision_link, g_pending_command_seq, &command);
        g_command_retry_count++;
        g_last_command_ms = now_ms;
      }
      else
      {
        g_command_pending = 0u;
        g_ack_timeout_count++;
        g_last_command_ms = now_ms;
      }
    }

    /* PB0 stays on only while a valid observation is fresh. */
    HAL_GPIO_WritePin(VISION_LINK_LED_GPIO_Port, VISION_LINK_LED_Pin,
                      vl_hal_observation_is_fresh(&g_vision_link, now_ms, 150u)
                      ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* PB1 gives a 150 ms pulse for each matched successful ACK. */
    HAL_GPIO_WritePin(VISION_ACTIVITY_LED_GPIO_Port, VISION_ACTIVITY_LED_Pin,
                      (int32_t)(g_ack_led_until_ms - now_ms) > 0
                      ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  vl_hal_uart_rx_cplt_isr(&g_vision_link, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  vl_hal_uart_error_isr(&g_vision_link, huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
