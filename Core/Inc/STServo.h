/**
 * @file STServo.h
 * @brief ST Series Serial Servo Control for STM32F707ZI
 * @author Adapted for STM32F707ZI
 * @date 2025
 * 
 * This provides control for STS3215 and other ST series servos
 * using UART IT communication on STM32F707ZI with HAL drivers.
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
// Baud rate definitions
#define SMS_STS_1M                  0
#define SMS_STS_0_5M                1
#define SMS_STS_250K                2
#define SMS_STS_128K                3
#define SMS_STS_115200              4
#define SMS_STS_76800               5
#define SMS_STS_57600               6
#define SMS_STS_38400               7

// Memory table definitions
// -------EPROM(Read Only)--------
#define SMS_STS_MODEL_L             3
#define SMS_STS_MODEL_H             4

// -------EPROM(Read/Write)--------
#define SMS_STS_ID                  5
#define SMS_STS_BAUD_RATE           6
#define SMS_STS_MIN_ANGLE_LIMIT_L   9
#define SMS_STS_MIN_ANGLE_LIMIT_H   10
#define SMS_STS_MAX_ANGLE_LIMIT_L   11
#define SMS_STS_MAX_ANGLE_LIMIT_H   12
#define SMS_STS_CW_DEAD             26
#define SMS_STS_CCW_DEAD            27
#define SMS_STS_OFS_L               31
#define SMS_STS_OFS_H               32
#define SMS_STS_MODE                33

// -------SRAM(Read/Write)--------
#define SMS_STS_TORQUE_ENABLE       40
#define SMS_STS_ACC                 41
#define SMS_STS_GOAL_POSITION_L     42
#define SMS_STS_GOAL_POSITION_H     43
#define SMS_STS_GOAL_TIME_L         44
#define SMS_STS_GOAL_TIME_H         45
#define SMS_STS_GOAL_SPEED_L        46
#define SMS_STS_GOAL_SPEED_H        47
#define SMS_STS_LOCK                55

// -------SRAM(Read Only)--------
#define SMS_STS_PRESENT_POSITION_L  56
#define SMS_STS_PRESENT_POSITION_H  57
#define SMS_STS_PRESENT_SPEED_L     58
#define SMS_STS_PRESENT_SPEED_H     59
#define SMS_STS_PRESENT_LOAD_L      60
#define SMS_STS_PRESENT_LOAD_H      61
#define SMS_STS_PRESENT_VOLTAGE     62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING              66
#define SMS_STS_PRESENT_CURRENT_L   69
#define SMS_STS_PRESENT_CURRENT_H   70

// Protocol constants
#define ST_SERVO_FRAME_HEADER       0xFF
#define ST_SERVO_FRAME_HEADER2      0xFF
#define ST_SERVO_BROADCAST_ID       0xFE
#define ST_SERVO_MAX_BUFFER_SIZE    0xFF
#define ST_SERVO_DEFAULT_RSP_SIZE   6
#define ST_SERVO_RD_RSP_SIZE        8

// Instruction set
#define ST_INST_PING                0x01
#define ST_INST_READ                0x02
#define ST_INST_WRITE               0x03
#define ST_INST_REG_WRITE           0x04
#define ST_INST_ACTION              0x05
#define ST_INST_SYNC_READ           0x82
#define ST_INST_SYNC_WRITE          0x83

// Application constants
#define MAX_SERVO_ID                21            // Max 30 for sync write command
#define RX_RING_BUFFER_SIZE         512           // Must be a power of 2
#define SERVO_RX_TIMEOUT            2             // millisecond

/*--------------- hw_error_flags ---------------*/
// Maps to the error flags received from the motor
#define ST_ERROR_NONE               0x00
#define ST_ERROR_VOLTAGE            0x01
#define ST_ERROR_ANGLE_LIMIT        0x02
#define ST_ERROR_OVERHEATING        0x04
#define ST_ERROR_RANGE              0x08
#define ST_ERROR_CHECKSUM           0x10
#define ST_ERROR_OVERLOAD           0x20
#define ST_ERROR_INSTRUCTION        0x40
/*----------------------------------------------*/

/*--------------- sw_error_flags ---------------*/
#define SERVO_ERROR_NONE            0x00
#define SERVO_PING_TX_FAILURE       0x01
#define SERVO_PING_RX_FAILURE       0x02
#define SERVO_SYNC_TX_OVERFLOW      0x04
#define SERVO_SYNC_TX_FAILURE       0x08
#define SERVO_RD_TX_FAILURE         0x10
#define SERVO_RD_RX_FAILURE         0x20
/*----------------------------------------------*/

/*--------------- status_flags -----------------*/
#define MOTOR_IS_ONLINE             0x01
#define MOTOR_IS_MOVING             0x02
/*----------------------------------------------*/

#define RX_RING_MSK                 (RX_RING_BUFFER_SIZE - 1)

// Push one byte into rx->buffer, advance tail with wrap
#define RING_PUSH(hrx_, byte_)                         \
  do {                                               \
    (hrx_)->buffer[(hrx_)->tail] = (byte_);             \
    (hrx_)->tail = ((hrx_)->tail + 1) & RX_RING_MSK;   \
  } while (0)

// pop one byte from rx->buffer, advance head with wrap
#define RING_POP(hrx_, byte_)                          \
  do {                                               \
    (byte_) = (hrx_)->buffer[(hrx_)->head];             \
    (hrx_)->head = ((hrx_)->head + 1) & RX_RING_MSK;   \
  } while (0)

// Set software error flag to every online servo
#define SERVO_SET_ERROR_ALL_ONLINE(flag_)                      \
  do {                                                       \
    for (uint8_t _id = 0; _id <= MAX_SERVO_ID; ++_id) {        \
      if (servo_data[_id].status_flags & MOTOR_IS_ONLINE)      \
        servo_data[_id].sw_error_flags |= (flag_);         \
    }                                                          \
  } while (0)


/* Public typedef  ----------------------------------------------------------*/
// Packet structure
typedef struct {
  uint8_t header[2];      // 0xFF, 0xFF
  uint8_t id;             // Servo ID
  uint8_t length;         // Length of parameters + 2
  uint8_t instruction;    // Instruction code
  uint8_t parameters[ST_SERVO_MAX_BUFFER_SIZE - 5];  // Parameters
  // Checksum appeneded to the end of parameters
} __attribute__((packed)) STServo_Packet_t;

// Servo serial receive handle
typedef struct {
  uint8_t            byte;
  uint8_t            info[4];                       // FF FF ID LEN
  volatile uint8_t   info_index;
  volatile uint8_t   data_len;                      // LEN after 4 byte info complete
  uint8_t            buffer[RX_RING_BUFFER_SIZE];   // Ring buffer to store msg
  volatile uint16_t  head;
  volatile uint16_t  tail;
  uint8_t            tmp_frame[32];
} STServo_RxHandle_t;

// Servo data handle structure
typedef struct {
  UART_HandleTypeDef  *huart;
  uint32_t            timeout_ms;
  bool                initialized;
  osSemaphoreId_t     txDone;
  osMessageQueueId_t  rxQueue;
  STServo_RxHandle_t  hrx;
} STServo_Handle_t;

// Servo get status structure
typedef struct {
  uint8_t status_flags;
  uint8_t hw_error_flags;  // hardware error flags received from the servo
  uint8_t sw_error_flags;  // servo software error flags
  int16_t position;        // present raw position
  int16_t speed;           // present raw speed (steps / second)
  int16_t load;            // present load (0.1% PWM duty cycle)
  uint8_t voltage;         // present voltage (0.1V)
  uint8_t temperature;     // C
  int16_t current;         // present current (6.5mA)
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


/* Public function prototypes -----------------------------------------------*/
// Initialization
bool STServo_Init(STServo_Handle_t *handle, UART_HandleTypeDef *huart);
void STServo_DeInit(STServo_Handle_t *handle);

// Basic servo control
STServo_Status_t STServo_WritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position,
                                       uint16_t speed, uint16_t time, uint8_t acceleration);

// Advanced servo control
STServo_Status_t STServo_SyncWritePosition(STServo_Handle_t *handle);

// Feedback functions
STServo_Status_t STServo_ReadPosition(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadSpeed(STServo_Handle_t *handle, uint8_t id);
STServo_Status_t STServo_ReadLoad(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadVoltage(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadTemperature(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadCurrent(STServo_Handle_t *handle, uint8_t id);
bool STServo_IsMoving(STServo_Handle_t *handle, uint8_t id);

// Utility functions
bool STServo_Ping(STServo_Handle_t *handle, uint8_t id);

// Communication functions
void STServo_ReadFromISR(STServo_Handle_t *handle);
STServo_Error_t STServo_GetLastError(STServo_Handle_t *handle);
const char* STServo_GetErrorString(STServo_Error_t error);


/* Public variables ---------------------------------------------------------*/
extern STServo_Handle_t hservo;
extern STServo_Data_t servo_data[MAX_SERVO_ID];
extern STServo_Control_t servo_control[MAX_SERVO_ID];

#ifdef __cplusplus
}
#endif

#endif // _ST_SERVO_H 
