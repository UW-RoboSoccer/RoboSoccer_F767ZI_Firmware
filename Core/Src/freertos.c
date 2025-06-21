/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "imu_driver.h"
#include "analog_driver.h"
#include "error.h"
#include "iwdg.h"
#include "STServo.h"
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
/* USER CODE BEGIN Variables */
extern uint16_t filtered_adc1_buffer[ADC1_CHANNEL_COUNT];
extern volatile uint32_t imu_error_flags;
extern UART_HandleTypeDef huart1;
/* USER CODE END Variables */
/* Definitions for AnalogTask */
osThreadId_t AnalogTaskHandle;
const osThreadAttr_t AnalogTask_attributes = {
  .name = "AnalogTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GeneralTask0 */
osThreadId_t GeneralTask0Handle;
const osThreadAttr_t GeneralTask0_attributes = {
  .name = "GeneralTask0",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorRxQueue */
osMessageQueueId_t motorRxQueueHandle;
const osMessageQueueAttr_t motorRxQueue_attributes = {
  .name = "motorRxQueue"
};
/* Definitions for adcSem */
osSemaphoreId_t adcSemHandle;
const osSemaphoreAttr_t adcSem_attributes = {
  .name = "adcSem"
};
/* Definitions for motorTxSem */
osSemaphoreId_t motorTxSemHandle;
const osSemaphoreAttr_t motorTxSem_attributes = {
  .name = "motorTxSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartAnalogTask(void *argument);
void StartGeneralTask0(void *argument);
void StartImuTask(void *argument);
void StartMotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
  // Refresh watch dog timer
  // If any task hangs for more than 1 second, trigger a software reset
  HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  printf("Stack overflow in %s\n\r", pcTaskName);
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcSem */
  adcSemHandle = osSemaphoreNew(1, 1, &adcSem_attributes);

  /* creation of motorTxSem */
  motorTxSemHandle = osSemaphoreNew(1, 1, &motorTxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  (void)osSemaphoreAcquire(adcSemHandle, 0);
  (void)osSemaphoreAcquire(motorTxSemHandle, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motorRxQueue */
  motorRxQueueHandle = osMessageQueueNew (64, sizeof(uint8_t), &motorRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AnalogTask */
  AnalogTaskHandle = osThreadNew(StartAnalogTask, NULL, &AnalogTask_attributes);

  /* creation of GeneralTask0 */
  GeneralTask0Handle = osThreadNew(StartGeneralTask0, NULL, &GeneralTask0_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartAnalogTask */
/**
  * @brief  Function implementing the AnalogTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAnalogTask */
void StartAnalogTask(void *argument)
{
  /* USER CODE BEGIN StartAnalogTask */
  (void)argument;
  // Give the first second to imu task
  TickType_t xWakeTime = xTaskGetTickCount();
  printf("Running AnalogTask 1ms loop \n\r");
  /* Infinite loop */
  for(;;)
  {
    if (ADC1_ReadAverage(ADC1_FILTER_SIZE, FSR_ADC_TIMEOUT) != ADC_OK)  {
      RoboSoccer_errorHandler();
    }
    vTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(1));
  }
  /* USER CODE END StartAnalogTask */
}

/* USER CODE BEGIN Header_StartGeneralTask0 */
/**
* @brief Function implementing the GeneralTask0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGeneralTask0 */
void StartGeneralTask0(void *argument)
{
  /* USER CODE BEGIN StartGeneralTask0 */
  (void)argument;
  // Give the first second to imu task
  TickType_t xWakeTime = xTaskGetTickCount();
  printf("Running GeneralTask 100ms loop \r\n");
  /* Infinite loop */
  for(;;)
  {
    if (filtered_adc1_buffer[0] > 3000) {
      LD2_GPIO_Port->BSRR = (uint32_t)LD2_Pin;
    } else {
      LD2_GPIO_Port->BSRR = (uint32_t)LD2_Pin << 16;
    }
    // printf("adc1[0] Reading: %d \r\n", filtered_adc1_buffer[0]);
    vTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(100));
  }
  /* USER CODE END StartGeneralTask0 */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  (void)argument;
  TickType_t xWakeTime = xTaskGetTickCount();
  if (imu_sys_init() != IMU_OK) {
    RoboSoccer_errorHandler();
  }
  printf("Running ImuTask 1ms loop \n\r");
  /* Infinite loop */
  for(;;)
  {
    // Cyclically service sh2
    imu_service();

    vTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(1));
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  // Give the first second to imu task
  if (!STServo_Init(&hservo, &huart1)) {
    printf("Motor Init Failed \n\r");
  } else {
    printf("Motor Init Success \n\r");
  }

  if (!STServo_Ping(&hservo, 2)) {
    printf("Motor 2 NOT Found \n\r");
  } else {
    printf("Motor 2 Connected \n\r");
  }

  if (!STServo_Ping(&hservo, 4)) {
    printf("Motor 4 NOT Found \n\r");
  } else {
    printf("Motor 4 Connected \n\r");
  }

//  const int16_t position[5] = {0, 1024, 2048, 3072, 4095};
//  const uint16_t speed = 0;
//  const uint8_t acc = 0; // Ramp up as fast as possible
  /* Infinite loop */
  for(;;)
  {
//    for (uint8_t i = 0; i < 5; i++) {
//      if (STServo_WritePosition(&hservo, 2, position[i], speed, acc)) {
//        printf("Servo %u moved to %d\n\r", 2, position[i]);
//      } else {
//        STServo_Error_t err = STServo_GetLastError(&hservo);
//        printf("Servo %u error: %s\n\r", 2, STServo_GetErrorString(err));
//      }
//      if (STServo_WritePosition(&hservo, 4, position[i], speed, acc)) {
//        printf("Servo %u moved to %d\n\r", 4, position[i]);
//      } else {
//        STServo_Error_t err = STServo_GetLastError(&hservo);
//        printf("Servo %u error: %s\n\r", 4, STServo_GetErrorString(err));
//      }
    osDelay(800);

  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

