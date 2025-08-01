/*
 * error.h
 *
 *  Created on: May 30, 2025
 *      Author: Bowen
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

#include "stm32f7xx_hal.h"


/* Public defines  ----------------------------------------------------------*/
#define   MAX_ERROR_NUM       255

/*---------- sys_error_flags ----------*/
#define SYS_ERROR_NONE            0x00U
#define MCU_VDDA_LOW_ERROR        0x01U
#define MCU_VDDA_HIGH_ERROR       0x02U
#define MCU_TEMP_HIGH_ERROR       0x04U
/*-------------------------------------*/


/* Public variables ---------------------------------------------------------*/
extern volatile uint8_t sys_error_flags;


/* Public function prototypes -----------------------------------------------*/
void RoboSoccer_errorHandler(void);

#endif /* INC_ERROR_H_ */
