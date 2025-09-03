/**
 * @file STServo.h
 * @brief ST Series Serial Servo Control for STM32F707ZI
 * @date 2025
 * 
 * This provides control for STS3215 and other ST series servos
 * using UART_IT tx and UART_DMA rx com on STM32F767ZI with HAL drivers.
 */

#ifndef _ST_SERVO_H
#define _ST_SERVO_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public defines  ----------------------------------------------------------*/
// Application constants
#define MAX_SERVO_ID                21U            // Max 30 for sync write command
#define RX_RING_BUFFER_SIZE         512U           // Must be a power of 2
#define SERVO_RX_TIMEOUT            10U            // millisecond
#define ST_SERVO_MAX_BUFFER_SIZE    0xFFU

/*--------------- hw_error_flags ---------------*/
// Maps to the error flags received from the motor
#define ST_ERROR_NONE               0x00U
#define ST_ERROR_VOLTAGE            0x01U
#define ST_ERROR_ANGLE_LIMIT        0x02U
#define ST_ERROR_OVERHEATING        0x04U
#define ST_ERROR_RANGE              0x08U
#define ST_ERROR_CHECKSUM           0x10U
#define ST_ERROR_OVERLOAD           0x20U
#define ST_ERROR_INSTRUCTION        0x40U
/*----------------------------------------------*/

/*--------------- sw_error_flags ---------------*/
#define SERVO_ERROR_NONE            0x00U
#define SERVO_PING_TX_FAILURE       0x01U
#define SERVO_PING_RX_FAILURE       0x02U
#define SERVO_SYNC_TX_OVERFLOW      0x04U
#define SERVO_SYNC_TX_FAILURE       0x08U
#define SERVO_RD_TX_FAILURE         0x10U
#define SERVO_RD_RX_FAILURE         0x20U
#define SERVO_SYNC_RX_FAILURE       0x40U
#define SERVO_SYNC_RX_ID_ERROR      0x80U
/*----------------------------------------------*/

/*--------------- status_flags -----------------*/
#define MOTOR_IS_ONLINE             0x01U
#define MOTOR_IS_MOVING             0x02U
/*----------------------------------------------*/


/* Public typedef  ----------------------------------------------------------*/
// Packet structure
typedef struct {
  uint8_t header[2];                                 // 0xFF, 0xFF
  uint8_t id;                                        // Servo ID
  uint8_t length;                                    // Length of parameters + 2
  uint8_t instruction;                               // Instruction code
  uint8_t parameters[ST_SERVO_MAX_BUFFER_SIZE - 5U]; // Parameters
  // Checksum appeneded to the end of parameters
} __attribute__((packed)) STServo_Packet_t;

// Servo serial receive context struct
typedef struct {
  uint8_t            byte;
  uint8_t            info[4];                       // FF FF ID LEN
  volatile uint8_t   info_index;
  volatile uint8_t   data_len;                      // LEN after 4 byte info complete
  volatile uint8_t   data_index;                    // Index of payload bytes
  uint8_t            buffer[RX_RING_BUFFER_SIZE];   // Ring buffer to store msg
  volatile uint16_t  head;
  volatile uint16_t  tail;
  uint8_t            tmp_frame[32];

  __attribute__((aligned(32)))
  uint8_t            dma_buf[RX_RING_BUFFER_SIZE];  // Raw circular DMA buffer
  volatile uint16_t  dma_last;                      // Last read index in dma_buf
} STServo_RxCtx_t;

// Servo data handle structure
typedef struct {
  UART_HandleTypeDef  *huart;
  uint32_t            timeout_ms;
  bool                initialized;
  osSemaphoreId_t     txDone;
  osMessageQueueId_t  rxQueue;
  STServo_RxCtx_t     rxCtx;
} STServo_Handle_t;

// Servo get status structure
typedef struct {
  uint8_t status_flags;
  uint8_t hw_error_flags;  // hardware error flags received from the servo
  uint8_t sw_error_flags;  // servo software error flags
  int16_t position;        // present raw position
//  int16_t speed;           // present raw speed (steps / second)
//  int16_t load;            // present load (0.1% PWM duty cycle)
//  uint8_t voltage;         // present voltage (0.1V)
//  uint8_t temperature;     // C
//  int16_t current;         // present current (6.5mA)
} STServo_Data_t;

// Servo control structure
typedef struct {
  int16_t  goal_position;     // goal position (0-4095)
  uint16_t goal_speed;        // goal speed (0-4095)
  uint8_t  goal_acc;          // goal Acceleration (0 = fastest ramp up)
  uint16_t goal_time;         // ramp up time (0 = fasted ramp up)
} STServo_Control_t;

// Error handling
typedef enum {
  STSERVO_OK                  = 0U,
  STSERVO_ERROR_TIMEOUT       = 1U,
  STSERVO_ERROR_CHECKSUM      = 2U,
  STSERVO_ERROR_INVALID_PARAM = 3U,
  STSERVO_ERROR_COMM_FAILED   = 4U,
  STSERVO_ERROR_HARDWARE      = 5U
} STServo_Error_t;

// Function return handling
typedef enum {
  SERVO_OK        = 0U,
  SERVO_ERROR     = 1U
} STServo_Status_t;


/* Public variables ---------------------------------------------------------*/
extern STServo_Handle_t hservo;
extern STServo_Data_t servo_data[MAX_SERVO_ID];
extern STServo_Control_t servo_control[MAX_SERVO_ID];


/* Public function prototypes -----------------------------------------------*/
// Initialization
bool STServo_Init(STServo_Handle_t *handle, UART_HandleTypeDef *huart);
void STServo_DeInit(STServo_Handle_t *handle);

// Servo control
STServo_Status_t STServo_WritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position,
                                       uint16_t speed, uint16_t time, uint8_t acceleration);
STServo_Status_t STServo_SyncWritePosition(STServo_Handle_t *handle);

// Feedback functions
STServo_Status_t STServo_ReadPosition(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadSpeed(STServo_Handle_t *handle, uint8_t id);
STServo_Status_t STServo_ReadLoad(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadVoltage(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadTemperature(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadCurrent(STServo_Handle_t *handle, uint8_t id);
bool STServo_IsMoving(STServo_Handle_t *handle, uint8_t id);
STServo_Status_t STServo_SyncRead(STServo_Handle_t *handle);

// Utility functions
bool STServo_Ping(STServo_Handle_t *handle, uint8_t id);

// Communication functions
void STServo_ReadFromISR(STServo_Handle_t *handle);
STServo_Error_t STServo_GetLastError(STServo_Handle_t *handle);
const char* STServo_GetErrorString(STServo_Error_t error);


#ifdef __cplusplus
}
#endif

#endif // _ST_SERVO_H 
