/*
 * error.c
 *
 * Author: Bowen
 */

#include "error.h"
#include "main.h"

uint8_t errorCode_list[MAX_ERROR_NUM];
extern volatile uint16_t hardware_fault_flags;
extern volatile uint32_t bus_error_addr;
extern volatile uint32_t mem_access_error_addr;
extern volatile uint8_t adc1_error_flags;
extern volatile uint32_t imu_error_flags;

void RoboSoccer_errorHandler(void)
{
  // Return early if no error flags are set
  if ( (imu_error_flags | adc1_error_flags | hardware_fault_flags) == 0) {
    return;
  }

  for (;;) {
    //__disable_irq();
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

    HAL_Delay(500);
    //__enable_irq();
  }


}

