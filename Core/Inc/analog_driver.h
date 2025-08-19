/*
 * analog_driver.h
 *
 *
 * Author:    Bowen Zheng
 */

#ifndef INC_ANALOG_DRIVER_H_
#define INC_ANALOG_DRIVER_H_

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

/* Public defines  ----------------------------------------------------------*/
// ADC1 definitions
#define ADC1_CHANNEL_COUNT        4U
#define FSR1_CHANNEL_INDEX        0U
#define FSR2_CHANNEL_INDEX        1U
#define TEMPSENSOR_CHANNEL_INDEX  2U
#define VDDA_CHANNEL_INDEX        3U

// MCU operating conditions
#define VDDA_HIGH_THRESHOLD       3600U    // millivolt
#define VDDA_LOW_THRESHOLD        2400U    // millivolt
#define MCU_TEMP_HIGH_THRESHOLD   120U     // C

// Application constants
#define ADC1_FILTER_SIZE          6U
#define VDDA_3300                 3300U   // millivolt
#define NO_FILTER                 1U
#define ADC_TIMEOUT               2U      // millisecond

/*------------ error_flags ------------*/
#define ADC_ERROR_NONE            0x00U
#define ADC_START_ERROR           0x01U
#define ADC_TIMEOUT_ERROR         0x02U
#define ADC_INVALID_PARAM         0x04U
#define ADC_VREF_SENSOR_ERROR     0x08U
#define ADC_TEMP_SENSOR_ERROR     0x10U
/*-------------------------------------*/

/*------------ status_flags -----------*/
#define ADC_STATUS_UNKNOWN        0x00U
#define ADC_STATUS_INITIALIZED    0x01U
#define ADC_OPERATIONAL           0x02U
/*-------------------------------------*/


/* Public typedef  ----------------------------------------------------------*/
typedef enum {
  ADC_OK    = 0U,
  ADC_ERROR = 1U
} adc_status_t;

// ADC average context structure
typedef struct {
  ADC_HandleTypeDef* hadc;         // HAL ADC handle
  volatile uint16_t* dma_buf;      // Raw DMA buffer
  uint16_t*          filtered_buf; // Output buffer
  uint8_t            buf_size;     // Words in dma_buf (#channels)
  osSemaphoreId_t    sem;          // Signals “DMA complete”
  volatile uint8_t   error_flags;  // ADC peripheral error flags
  volatile uint8_t   status_flags; // ADC peripheral status flags
} ADC_AvgCtx_t;


/* Public variables ---------------------------------------------------------*/
extern ADC_AvgCtx_t adc1_avg_ctx;
extern __attribute__((aligned(32))) volatile uint16_t adc1_dma_buffer[ADC1_CHANNEL_COUNT];
extern uint16_t adc1_filtered_buffer[ADC1_CHANNEL_COUNT];
extern uint16_t VDDA_mv;
extern uint8_t MCU_TEMP_c;


/* Public function prototypes -----------------------------------------------*/
void ADC1_Init(void);
adc_status_t ADC_ReadAverage(ADC_AvgCtx_t* ctx, uint8_t filter_size, TickType_t timeout);
uint16_t ADC_ReadVdda(void);
uint8_t ADC_ReadTempSensor(uint16_t vdda_mv);


#endif /* INC_ANALOG_DRIVER_H_ */
