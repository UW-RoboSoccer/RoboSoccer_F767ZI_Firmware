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
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
extern UART_HandleTypeDef huart1;
extern volatile bool btn_pressed;
/* USER CODE END Variables */
/* Definitions for AnalogTask */
osThreadId_t AnalogTaskHandle;
uint32_t AnalogTaskBuffer[ 1024 ];
osStaticThreadDef_t AnalogTaskControlBlock;
const osThreadAttr_t AnalogTask_attributes = {
  .name = "AnalogTask",
  .cb_mem = &AnalogTaskControlBlock,
  .cb_size = sizeof(AnalogTaskControlBlock),
  .stack_mem = &AnalogTaskBuffer[0],
  .stack_size = sizeof(AnalogTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GeneralTask0 */
osThreadId_t GeneralTask0Handle;
uint32_t GeneralTask0Buffer[ 1024 ];
osStaticThreadDef_t GeneralTask0ControlBlock;
const osThreadAttr_t GeneralTask0_attributes = {
  .name = "GeneralTask0",
  .cb_mem = &GeneralTask0ControlBlock,
  .cb_size = sizeof(GeneralTask0ControlBlock),
  .stack_mem = &GeneralTask0Buffer[0],
  .stack_size = sizeof(GeneralTask0Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
uint32_t ImuTaskBuffer[ 2048 ];
osStaticThreadDef_t ImuTaskControlBlock;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .cb_mem = &ImuTaskControlBlock,
  .cb_size = sizeof(ImuTaskControlBlock),
  .stack_mem = &ImuTaskBuffer[0],
  .stack_size = sizeof(ImuTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
uint32_t MotorTaskBuffer[ 2048 ];
osStaticThreadDef_t MotorTaskControlBlock;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .cb_mem = &MotorTaskControlBlock,
  .cb_size = sizeof(MotorTaskControlBlock),
  .stack_mem = &MotorTaskBuffer[0],
  .stack_size = sizeof(MotorTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for motorRxQueue */
osMessageQueueId_t motorRxQueueHandle;
uint8_t motorRxQueueBuffer[ 32 * sizeof( uint8_t ) ];
osStaticMessageQDef_t motorRxQueueControlBlock;
const osMessageQueueAttr_t motorRxQueue_attributes = {
  .name = "motorRxQueue",
  .cb_mem = &motorRxQueueControlBlock,
  .cb_size = sizeof(motorRxQueueControlBlock),
  .mq_mem = &motorRxQueueBuffer,
  .mq_size = sizeof(motorRxQueueBuffer)
};
/* Definitions for adc1Sem */
osSemaphoreId_t adc1SemHandle;
osStaticSemaphoreDef_t adc1SemControlBlock;
const osSemaphoreAttr_t adc1Sem_attributes = {
  .name = "adc1Sem",
  .cb_mem = &adc1SemControlBlock,
  .cb_size = sizeof(adc1SemControlBlock),
};
/* Definitions for motorTxSem */
osSemaphoreId_t motorTxSemHandle;
osStaticSemaphoreDef_t motorTxSemControlBlock;
const osSemaphoreAttr_t motorTxSem_attributes = {
  .name = "motorTxSem",
  .cb_mem = &motorTxSemControlBlock,
  .cb_size = sizeof(motorTxSemControlBlock),
};
/* Definitions for ServoEvent */
osEventFlagsId_t ServoEventHandle;
const osEventFlagsAttr_t ServoEvent_attributes = {
  .name = "ServoEvent"
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
  /* creation of adc1Sem */
  adc1SemHandle = osSemaphoreNew(1, 1, &adc1Sem_attributes);

  /* creation of motorTxSem */
  motorTxSemHandle = osSemaphoreNew(1, 1, &motorTxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  (void)osSemaphoreAcquire(adc1SemHandle, 0);
  (void)osSemaphoreAcquire(motorTxSemHandle, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motorRxQueue */
  motorRxQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &motorRxQueue_attributes);

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

  /* Create the event(s) */
  /* creation of ServoEvent */
  ServoEventHandle = osEventFlagsNew(&ServoEvent_attributes);

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
  TickType_t xWakeTime = xTaskGetTickCount();
  ADC1_Init();
  printf("Running AnalogTask 1ms loop \n\r");
  /* Infinite loop */
  for(;;)
  {
    // Read from adc1
    if (ADC_ReadAverage(&adc1_avg_ctx, ADC1_FILTER_SIZE, ADC_TIMEOUT) != ADC_OK)  {
      RoboSoccer_errorHandler();
    }
    // Read Vref
    VDDA_mv = ADC_ReadVdda();
    MCU_TEMP_c = ADC_ReadTempSensor(VDDA_mv);
    if (adc1_filtered_buffer[0] > 1500) {
      LD2_GPIO_Port->BSRR = (uint32_t)LD2_Pin;
    } else {
      LD2_GPIO_Port->BSRR = (uint32_t)LD2_Pin << 16;
    }
    vTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(2));
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
  TickType_t xWakeTime = xTaskGetTickCount();
  printf("Running GeneralTask 100ms loop \r\n");
  /* Infinite loop */
  for(;;)
  {
//
//    printf("imu_timestamp: %ld \r\n ", imu_spi.rx_timestamp);
//    printf("adc1[0] Reading: %d \r\n",adc1_filtered_buffer[0]);
//    printf("adc1[1] Reading: %d \r\n",adc1_filtered_buffer[1]);
    vTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(1000));
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
  (void)argument;
  TickType_t xWakeTime = xTaskGetTickCount();
  // Give the first second to imu task
  if (!STServo_Init(&hservo, &huart1)) {
    printf("Motor Init Failed \n\r");
  } else {
    printf("Motor Init Success \n\r");
  }

  for (uint8_t id = 0; id < MAX_SERVO_ID; id++) {
    if (!STServo_Ping(&hservo, id)) {
      printf("Motor %d NOT Found \n\r" , id);
    } else {
      printf("Motor %d Connected \n\r", id);
    }
  }

  const int16_t position[5] = {0, 1024, 2048, 3072, 4095};
  const uint16_t speed = 4095;
  const uint8_t acc = 0; // Ramp up as fast as possible
  const uint16_t time = 0;
  uint8_t step = 0;
  /* Infinite loop */
  for(;;)
  {
    if (btn_pressed) {
      btn_pressed = 0;
      // Write control table
      servo_control[2].goal_acc = acc;
      servo_control[2].goal_position = position[step];
      servo_control[2].goal_speed = speed;
      servo_control[2].goal_time = time;

      servo_control[4].goal_acc = acc;
      servo_control[4].goal_position = position[4 - step];
      servo_control[4].goal_speed = speed;
      servo_control[4].goal_time = time;

      servo_control[8].goal_acc = acc;
      servo_control[8].goal_position = position[4 - step];
      servo_control[8].goal_speed = speed;
      servo_control[8].goal_time = time;


      servo_control[15].goal_acc = acc;
      servo_control[15].goal_position = position[step];
      servo_control[15].goal_speed = speed;
      servo_control[15].goal_time = time;


      servo_control[17].goal_acc = acc;
      servo_control[17].goal_position = position[4 - step];
      servo_control[17].goal_speed = speed;
      servo_control[17].goal_time = time;

      if (STServo_SyncWritePosition(&hservo) != SERVO_OK) {
        STServo_Error_t err = STServo_GetLastError(&hservo);
        printf("Servo %u error: %s\n\r", 0xFE, STServo_GetErrorString(err));
      }

      step = (step + 1) % 5;
    }

//    STServo_ReadPosition(&hservo, 2);
//    STServo_ReadPosition(&hservo, 4);
//    STServo_ReadPosition(&hservo, 8);
//    STServo_ReadPosition(&hservo, 15);
//    STServo_ReadPosition(&hservo, 17);
    if (STServo_SyncRead(&hservo) != SERVO_OK) {
      STServo_Error_t err = STServo_GetLastError(&hservo);
      printf("Servo %u error: %s\n\r", 0xFE, STServo_GetErrorString(err));
    }
    vTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(2));
  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

