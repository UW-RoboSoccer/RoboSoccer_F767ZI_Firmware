/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_driver.h"
#include "error.h"
#include <stdio.h>
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// @brief  Possible STM32 system reset causes
typedef enum reset_cause_e
{
  RESET_CAUSE_UNKNOWN = 0,
  RESET_CAUSE_LOW_POWER_RESET,
  RESET_CAUSE_WINDOW_WATCHDOG_RESET,
  RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
  RESET_CAUSE_SOFTWARE_RESET,
  RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
  RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
  RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
reset_cause_t reset_cause_get(void);
const char* reset_cause_get_name(reset_cause_t reset_cause);
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
  SCB_InvalidateICache();
  SCB_InvalidateDCache();

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n\n\n======================================== \n\rSTM32F7 Peripherals Initialized\n\r");
  reset_cause_t reset_cause = reset_cause_get();
  printf("The system reset cause is \"%s\"\n\r", reset_cause_get_name(reset_cause));
  printf("Start osKernel Init \n\r");

  // IWDG disabled for debugging
  //MX_IWDG_Init();
//  uint32_t  raw_ch1;
//  while (1) {
//
//    /* 1. Make sure the ADC is not already in DMA mode */
//    HAL_ADC_Stop_DMA(&hadc1);          /* harmless if DMA wasn’t running   */
//    ADC_ChannelConfTypeDef sConfig = {0};
//    sConfig.Channel = ADC_CHANNEL_6;
//    sConfig.Rank = 1;
//    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//     {
//       Error_Handler();
//     }
//    HAL_ADC_Start(&hadc2);
//    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//    raw_ch1 = HAL_ADC_GetValue(&hadc2);   /* Rank-1 (Channel 1) */
//
//    printf("ch6 Reading: %ld \r\n", raw_ch1);
//
//    HAL_ADC_Stop(&hadc1);                 /* release the peripheral */
//    HAL_Delay(500);
//  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief   Obtain the STM32 system reset cause
  * @return  The system reset cause
  */
reset_cause_t reset_cause_get(void)
{
  reset_cause_t reset_cause;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
    reset_cause = RESET_CAUSE_LOW_POWER_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
    reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    // This reset is induced by calling the ARM CMSIS NVIC_SystemReset() function
    reset_cause = RESET_CAUSE_SOFTWARE_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
    reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    reset_cause = RESET_CAUSE_BROWNOUT_RESET;
  }
  else {
    reset_cause = RESET_CAUSE_UNKNOWN;
  }

  // Clear all the reset flags or else they will remain set during future
  // resets until system power is fully removed.
  __HAL_RCC_CLEAR_RESET_FLAGS();

  return reset_cause;
}


/**
  * @brief    Obtain the system reset cause as an ASCII-printable name string
  *           from a reset cause type
  * @param    reset_cause The previously-obtained system reset cause
  * @return   A null-terminated ASCII name string describing the system reset cause
  */
const char* reset_cause_get_name(reset_cause_t reset_cause)
{
  const char *reset_cause_name = "TBD";

  switch (reset_cause) {
    case RESET_CAUSE_UNKNOWN:
      reset_cause_name = "UNKNOWN";
      break;
    case RESET_CAUSE_LOW_POWER_RESET:
      reset_cause_name = "LOW_POWER_RESET";
      break;
    case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
      reset_cause_name = "WINDOW_WATCHDOG_RESET";
      break;
    case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
      reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
      break;
    case RESET_CAUSE_SOFTWARE_RESET:
      reset_cause_name = "SOFTWARE_RESET";
      break;
    case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
      reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
      break;
    case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
      reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
      break;
    case RESET_CAUSE_BROWNOUT_RESET:
      reset_cause_name = "BROWNOUT_RESET (BOR)";
      break;
  }

  return reset_cause_name;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  RoboSoccer_errorHandler();
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
