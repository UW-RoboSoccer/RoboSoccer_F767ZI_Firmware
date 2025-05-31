/*
 * analog_driver.h
 *
 *
 * Author:    Bowen Zheng
 */

#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"


#define   ADC1_CHANNEL_COUNT      1U
#define   ADC1_FILTER_SIZE        5U
#define   FSR_ADC_TIMEOUT         1U      // millisecond

/*--------- adc1_error_flags ----------*/
#define   ADC1_START_ERROR         0x1;
#define   ADC1_TIMEOUT_ERROR       0x2;
/*------------------------------------*/

typedef enum {
  ADC_OK = 0 ,
  ADC_Error = 1
} adc_status_t;


adc_status_t ADC1_ReadAverage(uint8_t filter_size, TickType_t timeout_ms);

#endif /* INC_ADC_DRIVER_H_ */
