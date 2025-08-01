/**
  ******************************************************************************
  * @file       analog_driver.c
  * @brief      This file provides driver code for the ADC
  ******************************************************************************
  *
  */
#include "analog_driver.h"
#include "adc.h"
#include "main.h"
#include "error.h"

/* Private defines ---------------------------------------------------------*/
// Get current adcx resolution bits (6, 8 , 10 , 12)
#define ADC_GET_RESOLUTION_BITS(hadc_)                                                   \
    (uint8_t)(12U - ((( (hadc_)->Instance->CR1 & ADC_CR1_RES_Msk) >> ADC_CR1_RES_Pos) * 2U)); \

// Calibrate one averaged ADC sample to 3.3V based on the current VDDA/VREF
#define ADC_CALIBRATE_SAMPLE(raw, vdda_mv, resolution_bits)            \
  ( (uint16_t)(                                                    \
      ( ((uint16_t)(raw) << (12U - (resolution_bits)) )                     \
    * ( (1U << (resolution_bits)) - 1U )                                  \
    + ( ((uint32_t)(vdda_mv) * 4095U + 1650U) / 3300U >> 1 ))        \
    / ( ((uint32_t)(vdda_mv) * 4095U + 1650U) / 3300U )            \
  ))


/* Private variables ----------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern osSemaphoreId_t adc1SemHandle;
ADC_AvgCtx_t adc1_avg_ctx;

/*------------------ ADC Buffers -----------------*/
__attribute__((aligned(32)))
volatile uint16_t adc1_dma_buffer[ADC1_CHANNEL_COUNT];
uint16_t adc1_filtered_buffer[ADC1_CHANNEL_COUNT];
/*------------------------------------------------*/
uint16_t VDDA_mv;
uint8_t MCU_TEMP_c;


/* Function definitions -------------------------------------------------------*/
/**
 * @brief  ADC1 Peripheral and buffer Init
 */
void ADC1_Init(void)
{
  MX_ADC1_Init();
  adc1_avg_ctx.hadc         = &hadc1;
  adc1_avg_ctx.buf_size     = ADC1_CHANNEL_COUNT;
  adc1_avg_ctx.dma_buf      = adc1_dma_buffer;
  adc1_avg_ctx.filtered_buf = adc1_filtered_buffer;
  adc1_avg_ctx.sem = adc1SemHandle;

  for (uint8_t i = 0; i < ADC1_CHANNEL_COUNT; i++) {
    adc1_dma_buffer[i] = 0;
    adc1_filtered_buffer[i] = 0;
    // Implement with EEPROM recovery later -> return false if needed
    adc1_avg_ctx.error_flags = 0;
    adc1_avg_ctx.status_flags = 0;
  }

  adc1_avg_ctx.status_flags |= ADC_STATUS_INITIALIZED;
}


/**
 * @brief  Get ADC reading with moving average filter
 * @param  ADC_HandleTypeDef Pointer to ADC HAL struct
 * @param  ADC_AvgCtx_t Pointer to ADC Context struct
 * @param  filter_size Moving-average window
 * @param  timeout  ADC sampling timeout ms
 * @retval Is adc sampling successful
 */
adc_status_t ADC_ReadAverage(ADC_AvgCtx_t* ctx, uint8_t filter_size, TickType_t timeout)
{
  if (!(ctx->status_flags & ADC_STATUS_INITIALIZED)) {
    ctx->error_flags |= ADC_INVALID_PARAM;
    return ADC_ERROR;
  }

  uint32_t sum_buffer[ctx->buf_size];
  for (uint8_t i = 0; i < ctx->buf_size; i++) {
    sum_buffer[i] = 0;
  }

  for (uint8_t i = 0; i < filter_size; i++) {
    // Clean data cache before DMA write
    SCB_CleanDCache_by_Addr((uint32_t*)ctx->dma_buf, ctx->buf_size * sizeof(uint16_t));

    if (HAL_ADC_Start_DMA(ctx->hadc, (uint32_t*)ctx->dma_buf, ctx->buf_size) != HAL_OK) {
      ctx->error_flags |= ADC_START_ERROR;
      return ADC_ERROR;
    }
    // wait until ISR signals
    if (osSemaphoreAcquire(ctx->sem, timeout) != osOK) {
      ctx->error_flags |= ADC_START_ERROR;
      HAL_ADC_Stop_DMA(ctx->hadc);
      return ADC_ERROR;
    }
    HAL_ADC_Stop_DMA(ctx->hadc);

    // Invalidate data Cache after DMA read
    SCB_InvalidateDCache_by_Addr((uint32_t*)ctx->dma_buf, ctx->buf_size * sizeof(uint16_t));

    // Sum all the samples
    for (uint8_t ch = 0; ch < ctx->buf_size; ch++) {
      sum_buffer[ch] += ctx->dma_buf[ch];
    }
  }

  // Calculate filtered buffer
  for (uint8_t ch = 0; ch < ctx->buf_size; ch++) {
    ctx->filtered_buf[ch] = (uint16_t)(sum_buffer[ch] / filter_size);
  }

  ctx->status_flags |= ADC_OPERATIONAL;
  return ADC_OK;
}


/**
 * @brief  Get VDDA ADC reading
 * @retval VDDA in 1 mv
 */
uint16_t ADC_ReadVdda(void) {
  // If no readings have been made, read once
  if (!(adc1_avg_ctx.status_flags & ADC_OPERATIONAL)) {
    if (ADC_ReadAverage(&adc1_avg_ctx, NO_FILTER, ADC_TIMEOUT) == ADC_ERROR)
      return 0;
  }

  uint16_t vref_raw = adc1_filtered_buffer[VDDA_CHANNEL_INDEX];
  // Flag if sensor is faulty
  if (vref_raw == 0) {
    if (adc1_avg_ctx.status_flags & ADC_OPERATIONAL) {
      adc1_avg_ctx.error_flags |= ADC_VREF_SENSOR_ERROR;
    }
    return 0;
  }

  // Convert raw sample to 12 bit resolution
  uint8_t adc1_resolution = ADC_GET_RESOLUTION_BITS(&hadc1);
  vref_raw = vref_raw << (12U - adc1_resolution);

  // Vref_cal / 3300mv = Vref_raw / Vdda
  const uint16_t vref_cal = *VREFINT_CAL_ADDR_CMSIS;
  uint16_t vdda = (uint16_t)((uint32_t)(VDDA_3300 * vref_cal + (vref_raw >> 1)) / vref_raw);
  if (vdda <= VDDA_LOW_THRESHOLD) {
    sys_error_flags |= MCU_VDDA_LOW_ERROR;
    return vdda;
  }
  if (vdda >= VDDA_HIGH_THRESHOLD) {
    sys_error_flags |= MCU_VDDA_HIGH_ERROR;
    return vdda;
  }

  return vdda;
}


/**
 * @brief  Get MCU temperature sensor reading
 * @param  vdda_mv Current VDDA in millivolt
 * @retval MCU temperature in 1 celsius
 */
uint8_t ADC_ReadTempSensor(uint16_t vdda_mv)
{
  // If no readings have been made, read once
  if (!(adc1_avg_ctx.status_flags & ADC_OPERATIONAL)) {
    ADC_ReadAverage(&adc1_avg_ctx, NO_FILTER, ADC_TIMEOUT);
  }

  uint16_t vtemp_raw = adc1_filtered_buffer[TEMPSENSOR_CHANNEL_INDEX];
  // Flag if sensor is faulty
  if (vtemp_raw == 0) {
    if (adc1_avg_ctx.status_flags & ADC_OPERATIONAL) {
      adc1_avg_ctx.error_flags |= ADC_TEMP_SENSOR_ERROR;
    }
  }

  // Calibrate temp sensor reading to 3.3v as the interpolation values are taken at 3.3v
  uint8_t adc1_resolution = ADC_GET_RESOLUTION_BITS(&hadc1);
  vtemp_raw = ADC_CALIBRATE_SAMPLE(vtemp_raw, vdda_mv, adc1_resolution);

  const uint16_t temp30_cal = *TEMPSENSOR_CAL1_ADDR_CMSIS;
  const uint16_t temp110_cal = *TEMPSENSOR_CAL2_ADDR_CMSIS;
  uint16_t delta_cal = (temp110_cal - temp30_cal);

  // Linear interpolation T(C) = (((raw – V30) × (110-30) + round)/(V110 – V30)) + 30
  uint8_t temp_c = (uint8_t)( (((vtemp_raw - temp30_cal) * (110U - 30U) + (delta_cal >> 1U)) / delta_cal) + 30U );

  if (temp_c >= MCU_TEMP_HIGH_THRESHOLD) {
    sys_error_flags |= MCU_TEMP_HIGH_ERROR;
  }
  return temp_c;
}
