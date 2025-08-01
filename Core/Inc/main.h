/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define SPI_IMU_RST_Pin GPIO_PIN_0
#define SPI_IMU_RST_GPIO_Port GPIOF
#define SPI_IMU_CS_Pin GPIO_PIN_1
#define SPI_IMU_CS_GPIO_Port GPIOF
#define SPI_IMU_PS0_WAKE_Pin GPIO_PIN_2
#define SPI_IMU_PS0_WAKE_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define USB_OTG_HS_ULPI_RST_Pin GPIO_PIN_1
#define USB_OTG_HS_ULPI_RST_GPIO_Port GPIOC
#define ADC_FSR1_Pin GPIO_PIN_0
#define ADC_FSR1_GPIO_Port GPIOA
#define ADC_FSR2_Pin GPIO_PIN_1
#define ADC_FSR2_GPIO_Port GPIOA
#define SPI_IMU_MOSI_Pin GPIO_PIN_2
#define SPI_IMU_MOSI_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define WIFI_RX_Pin GPIO_PIN_6
#define WIFI_RX_GPIO_Port GPIOC
#define WIFI_TX_Pin GPIO_PIN_7
#define WIFI_TX_GPIO_Port GPIOC
#define MOTOR_TX_Pin GPIO_PIN_9
#define MOTOR_TX_GPIO_Port GPIOA
#define MOTOR_RX_Pin GPIO_PIN_10
#define MOTOR_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPI_IMU_CLK_Pin GPIO_PIN_10
#define SPI_IMU_CLK_GPIO_Port GPIOC
#define SPI_IMU_MISO_Pin GPIO_PIN_11
#define SPI_IMU_MISO_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define SPI_IMU_INT_Pin GPIO_PIN_0
#define SPI_IMU_INT_GPIO_Port GPIOE
#define SPI_IMU_INT_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */
#define PRINTF_TX_BUFFER_SIZE     512

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
