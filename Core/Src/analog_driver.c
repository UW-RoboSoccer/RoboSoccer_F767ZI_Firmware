/**
  ******************************************************************************
  * @file       analog_driver.c
  * @brief      This file provides driver code for the ADC sensors
  ******************************************************************************
  *
  * Author:    Bowen Zheng
  */

#include "analog_driver.h"

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern osSemaphoreId_t adcSemHandle;
/*------------------ FSR DMA Buffers -----------------*/
__attribute__((aligned(32))) volatile uint16_t adc1_dma_buffer[ADC1_CHANNEL_COUNT] = {0};
uint16_t filtered_adc1_buffer[ADC1_CHANNEL_COUNT] = {0};
/*----------------------------------------------------*/
volatile uint8_t adc1_error_flags = 0;


/**
  * @brief  Get ADC1 reading with moving average filter
  * @param  filter_size avg filter size word
  * @param  timeout_ms Timeout to capture each reading in millisecond
  * @retval Is adc transmission successful
  */
adc_status_t ADC1_ReadAverage(uint8_t filter_size, TickType_t timeout_ms)
{
  uint32_t sum_buffer[ADC1_CHANNEL_COUNT] = {0};
  for (uint8_t i = 0; i < filter_size; i++) {
    // Clean data cache before DMA write
    SCB_CleanDCache_by_Addr((uint32_t*)adc1_dma_buffer, ADC1_CHANNEL_COUNT*sizeof(uint16_t));

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, ADC1_CHANNEL_COUNT) != HAL_OK) {
      adc1_error_flags |= ADC1_START_ERROR;
      return ADC_Error;
    }
    // wait until ISR signals
    if (osSemaphoreAcquire(adcSemHandle, timeout_ms) != osOK) {
      adc1_error_flags |= ADC1_TIMEOUT_ERROR;
      HAL_ADC_Stop_DMA(&hadc1);
      return ADC_Error;
    }
    HAL_ADC_Stop_DMA(&hadc1);

    // Invalidate data Cache after DMA read
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc1_dma_buffer, ADC1_CHANNEL_COUNT * sizeof(uint16_t));

    for (uint8_t ch = 0; ch < ADC1_CHANNEL_COUNT; ch++) {
      sum_buffer[ch] += adc1_dma_buffer[ch];
    }
  }

  for (uint8_t ch = 0; ch < ADC1_CHANNEL_COUNT; ch++) {
    filtered_adc1_buffer[ch] = (uint16_t)(sum_buffer[ch] / filter_size);
  }
  return ADC_OK;
}

