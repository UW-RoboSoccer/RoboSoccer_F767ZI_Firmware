/**
  ******************************************************************************
  * @file   STServo.c
  * @brief  This file provides driver code for the ST Series Serial Servos
  * @date   2025
  ******************************************************************************
  *
  */
#include "STServo.h"
#include <string.h>


/* Private defines ---------------------------------------------------------*/
#define RX_RING_MSK                 (RX_RING_BUFFER_SIZE - 1)

// Push one byte into rx->buffer, advance tail with wrap
#define RING_PUSH(rxCtx_, byte_)                         \
  do {                                               \
    (rxCtx_)->buffer[(rxCtx_)->tail] = (byte_);             \
    (rxCtx_)->tail = ((rxCtx_)->tail + 1) & RX_RING_MSK;   \
  } while (0)

// pop one byte from rx->buffer, advance head with wrap
#define RING_POP(rxCtx_, byte_)                          \
  do {                                               \
    (byte_) = (rxCtx_)->buffer[(rxCtx_)->head];             \
    (rxCtx_)->head = ((rxCtx_)->head + 1) & RX_RING_MSK;   \
  } while (0)

// Set software error flag to every online servo
#define SERVO_SET_ERROR_ALL_ONLINE(flag_)                      \
  do {                                                       \
    for (uint8_t _id = 0; _id <= MAX_SERVO_ID; ++_id) {        \
      if (servo_data[_id].status_flags & MOTOR_IS_ONLINE)      \
        servo_data[_id].sw_error_flags |= (flag_);         \
    }                                                          \
  } while (0)

// Baud rate definitions
#define SMS_STS_1M                  0U
#define SMS_STS_0_5M                1U
#define SMS_STS_250K                2U
#define SMS_STS_128K                3U
#define SMS_STS_115200              4U
#define SMS_STS_76800               5U
#define SMS_STS_57600               6U
#define SMS_STS_38400               7U

// Memory table definitions
// -------EPROM(Read Only)--------
#define SMS_STS_MODEL_L             3U
#define SMS_STS_MODEL_H             4U

// -------EPROM(Read/Write)--------
#define SMS_STS_ID                  5U
#define SMS_STS_BAUD_RATE           6U
#define SMS_STS_MIN_ANGLE_LIMIT_L   9U
#define SMS_STS_MIN_ANGLE_LIMIT_H   10U
#define SMS_STS_MAX_ANGLE_LIMIT_L   11U
#define SMS_STS_MAX_ANGLE_LIMIT_H   12U
#define SMS_STS_CW_DEAD             26U
#define SMS_STS_CCW_DEAD            27U
#define SMS_STS_OFS_L               31U
#define SMS_STS_OFS_H               32U
#define SMS_STS_MODE                33U

// -------SRAM(Read/Write)--------
#define SMS_STS_TORQUE_ENABLE       40U
#define SMS_STS_ACC                 41U
#define SMS_STS_GOAL_POSITION_L     42U
#define SMS_STS_GOAL_POSITION_H     43U
#define SMS_STS_GOAL_TIME_L         44U
#define SMS_STS_GOAL_TIME_H         45U
#define SMS_STS_GOAL_SPEED_L        46U
#define SMS_STS_GOAL_SPEED_H        47U
#define SMS_STS_LOCK                55U

// -------SRAM(Read Only)--------
#define SMS_STS_PRESENT_POSITION_L  56U
#define SMS_STS_PRESENT_POSITION_H  57U
#define SMS_STS_PRESENT_SPEED_L     58U
#define SMS_STS_PRESENT_SPEED_H     59U
#define SMS_STS_PRESENT_LOAD_L      60U
#define SMS_STS_PRESENT_LOAD_H      61U
#define SMS_STS_PRESENT_VOLTAGE     62U
#define SMS_STS_PRESENT_TEMPERATURE 63U
#define SMS_STS_MOVING              66U
#define SMS_STS_PRESENT_CURRENT_L   69U
#define SMS_STS_PRESENT_CURRENT_H   70U

// Instruction set
#define ST_INST_PING                0x01U
#define ST_INST_READ                0x02U
#define ST_INST_WRITE               0x03U
#define ST_INST_REG_WRITE           0x04U
#define ST_INST_ACTION              0x05U
#define ST_INST_SYNC_READ           0x82U
#define ST_INST_SYNC_WRITE          0x83U

// Protocol constants
#define ST_SERVO_FRAME_HEADER       0xFFU
#define ST_SERVO_FRAME_HEADER2      0xFFU
#define ST_SERVO_BROADCAST_ID       0xFEU
#define ST_SERVO_DEFAULT_RSP_SIZE   6U
#define ST_SERVO_RD_RSP_SIZE        8U
#define ST_SERVO_SYNC_RD_RSP_SIZE   22U


/* Private variables ---------------------------------------------------------*/
static STServo_Error_t last_error = STSERVO_OK;
extern osSemaphoreId_t motorTxSemHandle;
extern osMessageQueueId_t motorRxQueueHandle;
STServo_Handle_t hservo;
// Servo feedback data
STServo_Data_t servo_data[MAX_SERVO_ID] = {0};
// Servo control table
STServo_Control_t servo_control[MAX_SERVO_ID] = {0};


/* Private function prototypes -----------------------------------------------*/
static bool STServo_SendPacket(STServo_Handle_t *handle, const uint8_t *packet, uint8_t length);
static bool STServo_ReceivePacket(STServo_Handle_t *handle, uint8_t *packet, uint8_t *length);
static bool STServo_WriteData(STServo_Handle_t *handle, uint8_t id, uint8_t address,
                              uint8_t instruction, const uint8_t *data, uint8_t length);
static bool STServo_ReadData(STServo_Handle_t *handle, uint8_t id, uint8_t address, uint8_t *data, uint8_t length);
static uint8_t STServo_CalculateChecksum(const uint8_t *packet, uint8_t length);
static bool STServo_ValidateChecksum(const uint8_t *packet, uint8_t length);
static uint8_t STServo_BuildPacket(STServo_Packet_t *packet, uint8_t id, uint8_t instruction,
                                   const uint8_t *parameters, uint8_t param_length);


/* Function definitions -------------------------------------------------------*/
/**
 * @brief Initialize the ST Servo handle
 * @param handle Pointer to servo handle
 * @param huart Pointer to UART handle
 * @return true if successful, false otherwise
 */
bool STServo_Init(STServo_Handle_t *handle, UART_HandleTypeDef *huart)
{
  if (!handle || !huart) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  handle->huart = huart;
  handle->txDone = motorTxSemHandle;
  handle->rxQueue = motorRxQueueHandle;

  if (handle->txDone == NULL || handle->rxQueue == NULL) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  handle->timeout_ms = SERVO_RX_TIMEOUT;
  memset(&handle->rxCtx, 0, sizeof(handle->rxCtx));



  // Enable uart receive ISR
  if (HAL_UART_Receive_IT(handle->huart, &handle->rxCtx.byte, 1) != HAL_OK) {
    last_error = STSERVO_ERROR_COMM_FAILED;
    return false;
  }
  last_error = STSERVO_OK;
  handle->initialized = true;
  return true;
}


/**
 * @brief Send packet via UART
 * @param handle Servo handle
 * @param packet Packet data
 * @param length Packet length
 * @return true if successful, false otherwise
 */
static bool STServo_SendPacket(STServo_Handle_t *handle, const uint8_t *packet, uint8_t length)
{
  if (!handle || !packet) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

//  uint8_t dbg_pkt[7];
//  for (uint8_t i = 0; i < 7; i++) {
//    dbg_pkt[i] = packet[i];
//  }

  HAL_StatusTypeDef status = HAL_UART_Transmit_IT(handle->huart, (uint8_t*)packet, length);
  if (status != HAL_OK) {
    last_error = STSERVO_ERROR_COMM_FAILED;
    return false;
  }

  if (osSemaphoreAcquire(handle->txDone, handle->timeout_ms)) {
    last_error = STSERVO_ERROR_TIMEOUT;
    return false;
  }

  return true;
}


/**
 * @brief Receive packet via UART ISR queue
 * @param handle Servo handle
 * @param packet Buffer for received packet
 * @param length Pointer to received length
 * @return true if successful, false otherwise
 */
static bool STServo_ReceivePacket(STServo_Handle_t *handle, uint8_t *packet, uint8_t *length)
{
  if (!handle || !packet || !length) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  // Wait for message to arrive and task yield
  uint8_t msg_len;
  if (osMessageQueueGet(handle->rxQueue, &msg_len, 0, handle->timeout_ms)) {
    last_error = STSERVO_ERROR_TIMEOUT;
    return false;
  }

  // Message arrived
  //uint8_t debug_pkt[msg_len];
  // Write caller buffer
  for (uint8_t i = 0; i < msg_len; i++) {
    RING_POP(&handle->rxCtx, packet[i]);
    //debug_pkt[i] = packet[i];
  }
  // Write caller length
  *length = msg_len;

  // Validate checksum
  if (!STServo_ValidateChecksum(packet, *length)) {
    last_error = STSERVO_ERROR_CHECKSUM;
    return false;
  }

  return true;
}


/**
 * @brief Write data to servo register
 * @param handle Servo handle
 * @param id Servo ID
 * @param address Register address
 * @param instruction Servo instruction
 * @param data Data to write
 * @param length Data length
 * @return true if successful, false otherwise
 */
static bool STServo_WriteData(STServo_Handle_t *handle, uint8_t id, uint8_t address,
                              uint8_t instruction, const uint8_t *data, uint8_t length)
{
  if (!handle || !data || length == 0) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t params[ST_SERVO_MAX_BUFFER_SIZE - 6];
  params[0] = address;
  memcpy(&params[1], data, length);

  STServo_Packet_t packet;
  uint8_t packet_length = STServo_BuildPacket(&packet, id, instruction, params, length + 1);

  return STServo_SendPacket(handle, (uint8_t*)&packet, packet_length);
}


/**
 * @brief Read data from servo register
 * @param handle Servo handle
 * @param id Servo ID
 * @param address Register address
 * @param data Buffer for read data
 * @param length Data length to read
 * @return true if successful, false otherwise
 */
static bool STServo_ReadData(STServo_Handle_t *handle, uint8_t id, uint8_t address,
                             uint8_t *data, uint8_t length)
{
  if (!handle || !data || length == 0) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t params[2];
  params[0] = address;
  params[1] = length;

  STServo_Packet_t packet;
  uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_READ, params, 2);

  if (!STServo_SendPacket(handle, (uint8_t*)&packet, packet_length)) {
    return false;
  }

  uint8_t response[ST_SERVO_MAX_BUFFER_SIZE];
  uint8_t response_length;
  if (!STServo_ReceivePacket(handle, response, &response_length)) {
    return false;
  }

  // Extract data from response (skip header, ID, length, error, checksum)
  if (response_length >= length + 6) {
    memcpy(data, &response[5], length);
    return true;
  }

  last_error = STSERVO_ERROR_INVALID_PARAM;
  return false;
}


/**
 * @brief Calculate checksum for packet
 * @param packet Packet data (excluding checksum)
 * @param length Packet length (excluding checksum)
 * @return Calculated checksum
 */
static uint8_t STServo_CalculateChecksum(const uint8_t *packet, uint8_t length)
{
  if (!packet || length < 4) {
    return 0;
  }

  uint8_t checksum = 0;
  // Skip header bytes (0xFF, 0xFF), start from ID
  for (uint8_t i = 2; i < length; i++) {
    checksum += packet[i];
  }

  return ~checksum;  // Bitwise NOT
}


/**
 * @brief Validate packet checksum
 * @param packet Complete packet including checksum
 * @param length Total packet length
 * @return true if checksum is valid, false otherwise
 */
static bool STServo_ValidateChecksum(const uint8_t *packet, uint8_t length)
{
  if (!packet || length < 5) {
    return false;
  }

  uint8_t calculated_checksum = STServo_CalculateChecksum(packet, length - 1);
  uint8_t received_checksum = packet[length - 1];

  return calculated_checksum == received_checksum;
}


/**
 * @brief Build a complete packet
 * @param packet Pointer to packet structure
 * @param id Servo ID
 * @param instruction Instruction code
 * @param parameters Parameter data
 * @param param_length Parameter length
 * @return Total packet length
 */
static uint8_t STServo_BuildPacket(STServo_Packet_t *packet, uint8_t id, uint8_t instruction,
                                   const uint8_t *parameters, uint8_t param_length)
{
  if (!packet) {
    return 0;
  }

  // Build packet
  packet->header[0] = ST_SERVO_FRAME_HEADER;
  packet->header[1] = ST_SERVO_FRAME_HEADER2;
  packet->id = id;
  packet->length = param_length + 2;  // instruction + checksum
  packet->instruction = instruction;

  // Copy parameters if provided
  if (parameters && param_length > 0) {
    memcpy(packet->parameters, parameters, param_length);
  }

  // Calculate total length
  uint8_t total_length = 4 + param_length + 1;  // header + id + length + instruction + params

  // Appened checksum to the end of parameter buffer
  packet->parameters[param_length] = STServo_CalculateChecksum((uint8_t*)packet, total_length);

  return total_length + 1;  // Include checksum in total length
}


/**
 * @brief Handle read data from ISR
 * @param handle Servo handle
 */
void STServo_ReadFromISR(STServo_Handle_t *handle)
{
  STServo_RxCtx_t *rx = &handle->rxCtx;

  // Build 4-byte message info
  if (rx->info_index < 4) {
    // Store info byte
    rx->info[rx->info_index] = rx->byte;
    rx->info_index += 1;

    switch (rx->info_index) {
      case 1:
        // Start read second header byte
        HAL_UART_Receive_IT(handle->huart, &rx->byte, 1);
        break;

      case 2:
        if (rx->info[0] != 0xFF || rx->info[1] != 0xFF) {
          rx->info[0] = rx->info[1];
          rx->info_index = 1;
        }
        // Start read ID byte
        HAL_UART_Receive_IT(handle->huart, &rx->byte, 1);
        break;

      case 3:
        // Start read Length
        HAL_UART_Receive_IT(handle->huart, &rx->byte, 1);
        break;

      case 4:
        // Length received
        rx->data_len = rx->info[3];
        if (rx->data_len == 0 || rx->data_len > ST_SERVO_MAX_BUFFER_SIZE - 4) {
          // bad length, restart
          rx->info_index = 0;
          HAL_UART_Receive_IT(handle->huart, &rx->byte, 1);
        } else {
          // Start read data
          HAL_UART_Receive_IT(handle->huart, rx->tmp_frame, rx->data_len);
        }
        return;
    }
    return;
  }

  // Data arrived
  // Push info to the ring buffer
  for (uint8_t i = 0; i < 4; ++i) {
    RING_PUSH(rx, rx->info[i]);
  }
  // Push data to the ring buffer
  for (uint8_t i = 0; i < rx->data_len; i++) {
    RING_PUSH(rx, rx->tmp_frame[i]);
  }
  // Notify a message is received and buffer this frame length
  uint8_t frame_len = rx->data_len + 4;
  osMessageQueuePut(handle->rxQueue, &frame_len, 0, 0);

  // Start check for next message
  rx->info_index = 0;
  HAL_UART_Receive_IT(handle->huart, &rx->byte, 1);
}



/**
 * @brief Deinitialize the ST Servo handle
 * @param handle Pointer to servo handle
 */
void STServo_DeInit(STServo_Handle_t *handle)
{
  if (handle) {
    handle->initialized = false;
    handle->huart = NULL;
  }
}


/**
 * @brief Ping servo to check if it's responding
 * @param handle Servo handle
 * @param id Servo ID
 * @return true if servo responds, false otherwise
 */
bool STServo_Ping(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized || id > MAX_SERVO_ID) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  STServo_Packet_t packet;
  uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_PING, NULL, 0);

  if (!STServo_SendPacket(handle, (uint8_t*)&packet, packet_length)) {
    servo_data[id].sw_error_flags |= SERVO_PING_TX_FAILURE;
    return false;
  }

  uint8_t response[ST_SERVO_DEFAULT_RSP_SIZE];
  uint8_t response_length;

  if (!STServo_ReceivePacket(handle, response, &response_length)) {
    servo_data[id].sw_error_flags |= SERVO_PING_RX_FAILURE;
    return false;
  }

  // Check id
  if (response[2] != id) {
    servo_data[id].sw_error_flags |= SERVO_PING_RX_FAILURE;
    return false;
  }

  // Load working condition
  // Format : 0xFF, 0xFF, ID, LEN, Flags, CheckSum
  servo_data[id].hw_error_flags = response[4];
  servo_data[id].status_flags |= MOTOR_IS_ONLINE;

  if (servo_data[id].hw_error_flags != 0) {
    last_error = STSERVO_ERROR_HARDWARE;
    return false;
  }
  return true;
}


/**
 * @brief Write position command to servo
 * @param handle Servo handle
 * @param id Servo ID (1-253)
 * @param position Target position (0-4095)
 * @param speed Movement speed (0-4095)
 * @param acceleration Acceleration (0-255)
 * @return STServo_Status_t
 */
STServo_Status_t STServo_WritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position,
                                       uint16_t speed, uint16_t time, uint8_t acceleration)
{
  if (!handle || !handle->initialized || id > MAX_SERVO_ID) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return SERVO_ERROR;
  }

  uint8_t params[7];
  params[0] = acceleration;             // Goal Acceleration
  params[1] = position & 0xFF;          // Position low byte
  params[2] = (position >> 8) & 0xFF;   // Position high byte
  params[3] = time & 0xFF;              // Time low byte (0 = max speed)
  params[4] = (time >> 8) & 0xFF;       // Time high byte
  params[5] = speed & 0xFF;             // Speed low byte
  params[6] = (speed >> 8) & 0xFF;      // Speed high byte

  if (!STServo_WriteData(handle, id, SMS_STS_ACC, ST_INST_WRITE, params, sizeof(params))) {
    return SERVO_ERROR;
  }

  uint8_t response[ST_SERVO_DEFAULT_RSP_SIZE];
  uint8_t response_length;

  if (!STServo_ReceivePacket(handle, response, &response_length)) {
    return SERVO_ERROR;
  }

  // Load working condition
  // Format : 0xFF, 0xFF, ID, LEN, Flags, CheckSum
  servo_data[id].hw_error_flags = response[4];
  if (servo_data[id].hw_error_flags != 0) {
    last_error = STSERVO_ERROR_HARDWARE;
    return SERVO_ERROR;
  }

  return SERVO_OK;
}


/**
 * @brief Sync write position command to servo
 * @param handle Servo handle
 * @note  This fn read servo_control and servo_data
 * @return STServo_Status_t
 */
STServo_Status_t STServo_SyncWritePosition(STServo_Handle_t *handle)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return SERVO_ERROR;
  }

  // buffer to store parameters and track its index
  uint8_t params[ST_SERVO_MAX_BUFFER_SIZE - 5];
  uint8_t index = 0;

  // Check buffer overflow (max 255 bytes per transaction)
  // 8 constant bytes per transaction + 8 bytes per servo
  // Max 30 servo sync write command can be send per session
  if (MAX_SERVO_ID > 30) {
    SERVO_SET_ERROR_ALL_ONLINE(SERVO_SYNC_TX_OVERFLOW);
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return SERVO_ERROR;
  }

  // Write parameters
  params[index++] = 0x07;       // Bytes per servo, exclude id
  for (uint8_t id = 0; id < MAX_SERVO_ID; id++) {
    // Skip if current servo is not online
    if (!(servo_data[id].status_flags & MOTOR_IS_ONLINE)) continue;

    // Load params per servo from servo control table
    STServo_Control_t *servo = &servo_control[id];
    params[index++] = id;                                   // Servo id
    params[index++] = servo->goal_acc;                      // Goal Acceleration
    params[index++] = servo->goal_position & 0xFF;          // Position low byte
    params[index++] = (servo->goal_position >> 8) & 0xFF;   // Position high byte
    params[index++] = servo->goal_time & 0xFF;              // Time low byte (0 = max speed)
    params[index++] = (servo->goal_time >> 8) & 0xFF;       // Time high byte
    params[index++] = servo->goal_speed & 0xFF;             // Speed low byte
    params[index++] = (servo->goal_speed >> 8) & 0xFF;      // Speed high byte
  }

  if (!STServo_WriteData(handle, ST_SERVO_BROADCAST_ID, SMS_STS_ACC, ST_INST_SYNC_WRITE, params, index)) {
    SERVO_SET_ERROR_ALL_ONLINE(SERVO_RD_TX_FAILURE);
    return SERVO_ERROR;
  }

  return SERVO_OK;
}


/**
 * @brief Read current position (0-4095) from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @note  This fn load data to servo_data
 * @return STServo_Status_t
 */
STServo_Status_t STServo_ReadPosition(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized || id > MAX_SERVO_ID) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return SERVO_ERROR;
  }

  static const uint8_t params_length = 2;
  if (!STServo_WriteData(handle, id, SMS_STS_PRESENT_POSITION_L, ST_INST_READ, &params_length, 1)) {
    servo_data[id].sw_error_flags |= SERVO_RD_TX_FAILURE;
    return SERVO_ERROR;
  }

  static uint8_t response[ST_SERVO_RD_RSP_SIZE];
  uint8_t response_length;

  if (!STServo_ReceivePacket(handle, response, &response_length)) {
    servo_data[id].sw_error_flags |= SERVO_RD_RX_FAILURE;
    return SERVO_ERROR;
  }

  // Check id
  if (response[2] != id) {
    servo_data[id].sw_error_flags |= SERVO_RD_RX_FAILURE;
    return SERVO_ERROR;
  }

  // Load data and working condition
  // Format : 0xFF, 0xFF, ID, LEN, Flags, data_L, data_H, CheckSum
  servo_data[id].hw_error_flags = response[4];
  servo_data[id].position = response[5] + (response[6] << 8);
  if (servo_data[id].hw_error_flags != 0) {
    last_error = STSERVO_ERROR_HARDWARE;
    return SERVO_ERROR;
  }

  return SERVO_OK;
}


/**
 * @brief Read current speed from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current speed or -1 on error
 */
int16_t STServo_ReadSpeed(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return -1;
  }

  uint8_t data[2];
  if (STServo_ReadData(handle, id, SMS_STS_PRESENT_SPEED_L, data, 2)) {
    return (int16_t)(data[0] | (data[1] << 8));
  }
  return -1;
}


/**
 * @brief Read current load (0.1% PWM duty cycle) from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @note  This fn load data to servo_data
 * @return STServo_Status_t
 */
//STServo_Status_t STServo_ReadLoad(STServo_Handle_t *handle, uint8_t id)
//{
//  if (!handle || !handle->initialized || id > MAX_SERVO_ID) {
//    last_error = STSERVO_ERROR_INVALID_PARAM;
//    return SERVO_ERROR;
//  }
//
//  static const uint8_t params_length = 2;
//  if (!STServo_WriteData(handle, id, SMS_STS_PRESENT_LOAD_L, ST_INST_READ, &params_length, 1)) {
//    servo_data[id].sw_error_flags |= SERVO_RD_TX_FAILURE;
//    return SERVO_ERROR;
//  }
//
//  static uint8_t response[ST_SERVO_RD_RSP_SIZE];
//  uint8_t response_length;
//
//  if (!STServo_ReceivePacket(handle, response, &response_length)) {
//    servo_data[id].sw_error_flags |= SERVO_RD_RX_FAILURE;
//    return SERVO_ERROR;
//  }
//
//  // Check id
//  if (response[2] != id) {
//    servo_data[id].sw_error_flags |= SERVO_RD_RX_FAILURE;
//    return SERVO_ERROR;
//  }
//
//  // Load data and working condition
//  // Format : 0xFF, 0xFF, ID, LEN, Flags, data_L, data_H, CheckSum
//  servo_data[id].hw_error_flags = response[4];
//  servo_data[id].load = (response[5] + (response[6] << 8));
//  if (servo_data[id].hw_error_flags != 0) {
//    last_error = STSERVO_ERROR_HARDWARE;
//    return SERVO_ERROR;
//  }
//
//  return SERVO_OK;
//}


/**
 * @brief Sync read servo feedback (position, speed, load, voltage, temperature, current)
 * @param handle Servo handle
 * @note  This fn load data to servo_data
 * @return STServo_Status_t
 */
STServo_Status_t STServo_SyncRead(STServo_Handle_t *handle)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return SERVO_ERROR;
  }

  // buffer to store parameters and track its index
  uint8_t params[ST_SERVO_MAX_BUFFER_SIZE - 5];
  uint8_t index = 0;

  // Write parameters
  params[index++] = SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L + 1;  // Data length
  for (uint8_t id = 0; id < MAX_SERVO_ID; id++) {
    // Skip if current servo is not online
    if (!(servo_data[id].status_flags & MOTOR_IS_ONLINE)) continue;
    params[index++] = id;                                                        // Servo id
  }

  if (!STServo_WriteData(handle, ST_SERVO_BROADCAST_ID, SMS_STS_PRESENT_POSITION_L,
                         ST_INST_SYNC_READ, params, index)) {
    SERVO_SET_ERROR_ALL_ONLINE(SERVO_SYNC_TX_FAILURE);
    return SERVO_ERROR;
  }

  /* Handle data feedback
   * Format: 0xFF, 0xFF, LEN, WorkCondi, Position_L, Position_H, Speed_L, Speed_H, Load_L, Load_H,
             Voltage, Temperature, DNC, DNC, movement flag, DNC, DNC, Current_L, Current_H, Checksum
  */
  for (uint8_t id = 0; id < MAX_SERVO_ID; id++) {
    // Skip if current servo is not online
    if (!(servo_data[id].status_flags & MOTOR_IS_ONLINE)) continue;

    uint8_t response[ST_SERVO_SYNC_RD_RSP_SIZE];
    uint8_t response_length;

    if (!STServo_ReceivePacket(handle, response, &response_length)) {
      servo_data[id].sw_error_flags |= SERVO_SYNC_RX_FAILURE;
      return SERVO_ERROR;
    }

    // Check id
    if (response[2] != id) {
      servo_data[id].sw_error_flags |= SERVO_SYNC_RX_FAILURE;
      return SERVO_ERROR;
    }

    // Load working condition
    servo_data[id].hw_error_flags = response[4];
    if (servo_data[id].hw_error_flags != 0) {
      last_error = STSERVO_ERROR_HARDWARE;
      return SERVO_ERROR;
    }

    // Load data feedback
    servo_data[id].position = (response[5] + (response[6] << 8));
//    servo_data[id].speed = (response[7] + (response[8] << 8));
//    servo_data[id].load = (response[9] + (response[10] << 8));
//    servo_data[id].voltage = response[11];
//    servo_data[id].temperature = response[12];
//    servo_data[id].status_flags = response[15] ? (servo_data[id].status_flags | MOTOR_IS_MOVING)
//                                               : (servo_data[id].status_flags & ~MOTOR_IS_MOVING);
//    servo_data[id].current = (response[18] + (response[19] << 8));
  }

  return SERVO_OK;
}


/**
 * @brief Read voltage from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Voltage in 0.1V units or -1 on error
 */
int16_t STServo_ReadVoltage(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return -1;
  }

  uint8_t data;
  if (STServo_ReadData(handle, id, SMS_STS_PRESENT_VOLTAGE, &data, 1)) {
    return (int16_t)data;
  }
  return -1;
}


/**
 * @brief Read temperature from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Temperature in Celsius or -1 on error
 */
int16_t STServo_ReadTemperature(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return -1;
  }

  uint8_t data;
  if (STServo_ReadData(handle, id, SMS_STS_PRESENT_TEMPERATURE, &data, 1)) {
    return (int16_t)data;
  }

  return -1;
}


/**
 * @brief Read current from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current in mA or -1 on error
 */
int16_t STServo_ReadCurrent(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return -1;
  }

  uint8_t data[2];
  if (STServo_ReadData(handle, id, SMS_STS_PRESENT_CURRENT_L, data, 2)) {
    return (int16_t)(data[0] | (data[1] << 8));
  }
  return -1;
}


/**
 * @brief Check if servo is moving
 * @param handle Servo handle
 * @param id Servo ID
 * @return true if moving, false if stopped or error
 */
bool STServo_IsMoving(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t data;
  if (STServo_ReadData(handle, id, SMS_STS_MOVING, &data, 1)) {
    return data != 0;
  }
  return false;
}


/**
 * @brief Get last error
 * @param handle Servo handle
 * @return Last error code
 */
STServo_Error_t STServo_GetLastError(STServo_Handle_t *handle)
{
  (void)handle;  // Unused parameter
  return last_error;
}


/**
 * @brief Get error string
 * @param error Error code
 * @return Error description string
 */
const char* STServo_GetErrorString(STServo_Error_t error)
{
  switch (error) {
    case STSERVO_OK: return "No error";
    case STSERVO_ERROR_TIMEOUT: return "Servo communication timeout";
    case STSERVO_ERROR_CHECKSUM: return "Servo checksum error";
    case STSERVO_ERROR_INVALID_PARAM: return "Servo invalid parameter";
    case STSERVO_ERROR_COMM_FAILED: return "Servo communication failed";
    case STSERVO_ERROR_HARDWARE: return "Servo hardware Fault";
    default: return "Unknown error";
  }
}
