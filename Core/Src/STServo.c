/**
 * @file STServo.c
 * @brief ST Series Serial Servo Control Implementation
 * @author Adapted for STM32F707ZI and FreeRTOS
 * @date 2025
 */

#include "STServo.h"
//#include "STServo_Protocol.h"
#include <string.h>

// Private variables
static STServo_Error_t last_error = STSERVO_OK;
extern osSemaphoreId_t motorTxSemHandle;
extern osMessageQueueId_t motorRxQueueHandle;
STServo_Handle_t hservo;
STServo_Status_t servo_status[MAX_SERVO_ID];

// Private function prototypes
static bool STServo_SendPacket(STServo_Handle_t *handle, const uint8_t *packet, uint8_t length);
static bool STServo_ReceivePacket(STServo_Handle_t *handle, uint8_t *packet, uint8_t *length);
static bool STServo_WriteData(STServo_Handle_t *handle, uint8_t id, uint8_t address, const uint8_t *data, uint8_t length);
static bool STServo_ReadData(STServo_Handle_t *handle, uint8_t id, uint8_t address, uint8_t *data, uint8_t length);
static uint8_t STServo_CalculateChecksum(const uint8_t *packet, uint8_t length);
static bool STServo_ValidateChecksum(const uint8_t *packet, uint8_t length);
static uint8_t STServo_BuildPacket(STServo_Packet_t *packet, uint8_t id, uint8_t instruction,
                                   const uint8_t *parameters, uint8_t param_length);

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

  handle->timeout_ms = 1000;
  memset(&handle->hrx, 0, sizeof(handle->hrx));
  // Enable uart receive ISR
  if (HAL_UART_Receive_IT(handle->huart, &handle->hrx.byte, 1) != HAL_OK) {
    last_error = STSERVO_ERROR_COMM_FAILED;
  }

  last_error = STSERVO_OK;
  handle->initialized = true;
  return true;
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
 * @brief Write position command to servo
 * @param handle Servo handle
 * @param id Servo ID (1-253)
 * @param position Target position (0-4095)
 * @param speed Movement speed (0-4095)
 * @param acceleration Acceleration (0-255)
 * @return true if successful, false otherwise
 */
bool STServo_WritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position,
                           uint16_t speed, uint16_t time, uint8_t acceleration)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t params[7];
  params[0] = acceleration;
  params[1] = position & 0xFF;          // Position low byte
  params[2] = (position >> 8) & 0xFF;   // Position high byte
  params[3] = time & 0xFF;              // Time low byte (0 = max speed)
  params[4] = (time >> 8) & 0xFF;       // Time high byte
  params[5] = speed & 0xFF;             // Speed low byte
  params[6] = (speed >> 8) & 0xFF;      // Speed high byte

  if (!STServo_WriteData(handle, id, SMS_STS_ACC, params, sizeof(params))) {
    return false;
  }

  uint8_t response[ST_SERVO_DEFAULT_RSP_SIZE];
  uint8_t response_length;

  if (!STServo_ReceivePacket(handle, response, &response_length)) {
    return false;
  }

  // Load working condition
  // Format : 0xFF, 0xFF, ID, LEN, Flags, CheckSum
  servo_status[id].hw_error_flags = response[4];
  if (servo_status[id].hw_error_flags != 0) {
    last_error = STSERVO_ERROR_HARDWARE;
    return false;
  }

  return true;
}

/**
 * @brief Write speed command to servo (wheel mode)
 * @param handle Servo handle
 * @param id Servo ID
 * @param speed Speed (-4095 to 4095, negative = reverse)
 * @param acceleration Acceleration (0-255)
 * @return true if successful, false otherwise
 */
bool STServo_WriteSpeed(STServo_Handle_t *handle, uint8_t id, int16_t speed, uint8_t acceleration)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t speed_data[2];
  speed_data[0] = speed & 0xFF;
  speed_data[1] = (speed >> 8) & 0xFF;

  return STServo_WriteData(handle, id, SMS_STS_ACC, &acceleration, 1) &&
         STServo_WriteData(handle, id, SMS_STS_GOAL_SPEED_L, speed_data, 2);
}

/**
 * @brief Enable or disable servo torque
 * @param handle Servo handle
 * @param id Servo ID
 * @param enable true to enable torque, false to disable
 * @return true if successful, false otherwise
 */
bool STServo_EnableTorque(STServo_Handle_t *handle, uint8_t id, bool enable)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t torque_enable = enable ? 1 : 0;
  return STServo_WriteData(handle, id, SMS_STS_TORQUE_ENABLE, &torque_enable, 1);
}

/**
 * @brief Read current position from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current position (0-4095) or -1 on error
 */
int16_t STServo_ReadPosition(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return -1;
  }

  uint8_t data[2];
  if (STServo_ReadData(handle, id, SMS_STS_PRESENT_POSITION_L, data, 2)) {
    return (int16_t)(data[0] | (data[1] << 8));
  }
  return -1;
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
 * @brief Read current load from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current load or -1 on error
 */
int16_t STServo_ReadLoad(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return -1;
  }

  uint8_t data[2];
  if (STServo_ReadData(handle, id, SMS_STS_PRESENT_LOAD_L, data, 2)) {
    return (int16_t)(data[0] | (data[1] << 8));
  }
  return -1;
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
 * @brief Ping servo to check if it's responding
 * @param handle Servo handle
 * @param id Servo ID
 * @return true if servo responds, false otherwise
 */
bool STServo_Ping(STServo_Handle_t *handle, uint8_t id)
{
  if (!handle || !handle->initialized) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  STServo_Packet_t packet;
  uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_PING, NULL, 0);

  if (!STServo_SendPacket(handle, (uint8_t*)&packet, packet_length)) {
    return false;
  }

  uint8_t response[ST_SERVO_DEFAULT_RSP_SIZE];
  uint8_t response_length;

  if (!STServo_ReceivePacket(handle, response, &response_length)) {
    return false;
  }

  // Check id
  if (response[2] != id) {
    return false;
  }

  // Load working condition
  // Format : 0xFF, 0xFF, ID, LEN, Flags, CheckSum
  servo_status[id].hw_error_flags = response[4];
  servo_status[id].status_flags |= MOTOR_IS_ONLINE;

  if (servo_status[id].hw_error_flags != 0) {
    last_error = STSERVO_ERROR_HARDWARE;
    return false;
  }
  return true;
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

// Private function implementations

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

  uint8_t dbg_pkt[7];
  for (uint8_t i = 0; i < 7; i++) {
    dbg_pkt[i] = packet[i];
  }

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
  uint8_t debug_pkt[msg_len];
  // Write caller buffer
  for (uint8_t i = 0; i < msg_len; i++) {
    RING_POP(&handle->hrx, packet[i]);
    debug_pkt[i] = packet[i];
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
 * @param data Data to write
 * @param length Data length
 * @return true if successful, false otherwise
 */
static bool STServo_WriteData(STServo_Handle_t *handle, uint8_t id, uint8_t address, const uint8_t *data, uint8_t length)
{
  if (!handle || !data || length == 0) {
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
  }

  uint8_t params[ST_SERVO_MAX_BUFFER_SIZE - 6];
  params[0] = address;
  memcpy(&params[1], data, length);

  STServo_Packet_t packet;
  uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_WRITE, params, length + 1);

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
static bool STServo_ReadData(STServo_Handle_t *handle, uint8_t id, uint8_t address, uint8_t *data, uint8_t length)
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
 * @brief Handle read data from ISR
 * @param handle Servo handle
 */
void STServo_ReadFromISR(STServo_Handle_t *handle)
{
  STServo_RxHandle_t *rx = &handle->hrx;

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
 * @brief Calculate checksum for packet
 * @param packet Packet data (excluding checksum)
 * @param length Packet length (excluding checksum)
 * @return Calculated checksum
 */
uint8_t STServo_CalculateChecksum(const uint8_t *packet, uint8_t length)
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
bool STServo_ValidateChecksum(const uint8_t *packet, uint8_t length)
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
uint8_t STServo_BuildPacket(STServo_Packet_t *packet, uint8_t id, uint8_t instruction,
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


