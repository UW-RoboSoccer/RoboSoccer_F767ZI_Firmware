/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F7xx_IT_H
#define __STM32F7xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

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
/*------------- hard_fault_flags ---------------*/
#define   VECTOR_TABLE_READ_FAULT         0x0001U
#define   FORCED_HARD_FAULT               0x0002U
#define   INSTRUCTION_ACCESS_VIOLATION    0x0004U
#define   DATA_ACCESS_VIOLATION           0x0008U
#define   INSTRUCTION_BUS_ERROR           0x0010U
#define   PRECISE_DATA_BUS_ERROR          0x0020U
#define   IMPRECISE_DATA_BUS_ERROR        0x0040U
#define   UNDEFINED_INSTRUCTION_ERROR     0x0080U
#define   INVALID_STATE_ERROR             0x0100U
#define   INVALID_PC_LOAD                 0x0200U
#define   NO_PROCESSOR                    0x0400U
#define   UNALIGNED_ACCESS                0x0800U
#define   DIVISION_BY_ZERO                0x1000U
/*----------------------------------------------*/


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void EXTI0_IRQHandler(void);
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void SPI3_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void USART6_IRQHandler(void);
void OTG_HS_EP1_OUT_IRQHandler(void);
void OTG_HS_EP1_IN_IRQHandler(void);
void OTG_HS_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F7xx_IT_H */
