/*
 * error.c
 *
 */

#include "error.h"
#include "main.h"
#include "analog_driver.h"
#include <stdio.h>


/* Private variables ----------------------------------------------------------*/
uint8_t errorCode_list[MAX_ERROR_NUM];
extern volatile uint16_t hard_fault_flags;
extern volatile uint32_t bus_error_addr;
extern volatile uint32_t mem_access_error_addr;
extern volatile uint8_t adc1_error_flags;
extern volatile uint32_t imu_error_flags;
volatile uint8_t sys_error_flags;


/* Function definitions -------------------------------------------------------*/
void RoboSoccer_errorHandler(void)
{

  for (;;) {
    printf("====== System Error  ======\r\n");
    printf("Hardware Fault Flags   : 0x%04X\r\n", hard_fault_flags);
    printf("Bus Error Address      : 0x%08lX\r\n", bus_error_addr);
    printf("Mem Access Error Addr  : 0x%08lX\r\n", mem_access_error_addr);
    printf("ADC1 Error Flags       : 0x%02X\r\n", adc1_avg_ctx.error_flags);
    printf("IMU Error Flags        : 0x%08lX\r\n", imu_error_flags);
    printf("===========================\r\n");
    __disable_irq();
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

    HAL_Delay(500);
    //__enable_irq();
  }

}

