/*
 * imu_driver.h
 *
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "sh2_hal.h"

/* Public defines  ----------------------------------------------------------*/
// SHTP protocol constants
#define SHTP_HEADER_SIZE               4U
#define SHTP_PKT_LEN_MSK               0x7FFFU

// BNO085 report rate definitions
#define BNO08X_REPORT_1000HZ           1000U       // microsecond
#define BNO08X_REPORT_500HZ            2000U       // microsecond
#define BNO08X_REPORT_400HZ            2500U       // microsecond

// BNO085 SPI constants
#define SPI_READ_SHTP_HEADER_TIMEOUT   10U         // millisecond
#define SPI_READ_SHTP_DATA_TIMEOUT     10U         // millisecond
#define IMU_RESET_DELAY                10000U      // microsecond
#define IMU_START_RESET_DELAY          2000000U    // microsecond
#define IMU_START_INIT_DELAY           200000U     // microsecond

/*-------------- imu_error_flags ---------------*/
#define IMU_ERROR_NONE                 0x0000U
#define IMU_PERIPHERAL_FAILURE         0x0001U
#define IMU_COMMUNICATION_TIMEOUT      0x0002U
#define IMU_HEADER_DATA_ERROR          0x0004U
#define IMU_TRANSMIT_ERROR             0x0008U
#define IMU_SH2_FAILED_TO_OPEN         0x0010U
#define IMU_READ_HEADER_FAILURE        0x0020U
#define IMU_READ_PKT_FAILURE           0x0040U
#define IMU_SENSOR_DISABLE_FAILURE     0x0080U
#define IMU_SENSOR_ENABLE_FAILURE      0x0100U
#define IMU_SPI_COMPLETE_FSM_ERROR     0x0200U
#define IMU_FAILED_TO_BOOT             0x0400U
#define IMU_DECODE_SENSOR_EVENT_ERROR  0x0800U
#define IMU_INIT_SENSOR_EVENT_ERROR    0x1000U
/*----------------------------------------------*/

/*--------------- imu_hal_flags ----------------*/
#define IMU_HAL_OPENED                 0x01U
#define IMU_INT_SERVICED               0x02U
#define IMU_IN_RESET                   0x04U
#define IMU_RESET_OCCURRED             0x08U
/*----------------------------------------------*/

#define IMU_CS_SET_LOW()               SPI_IMU_CS_GPIO_Port->BSRR = (uint32_t)SPI_IMU_CS_Pin << 16U
#define IMU_CS_SET_HIGH()              SPI_IMU_CS_GPIO_Port->BSRR = (uint32_t)SPI_IMU_CS_Pin
#define IMU_RST_SET_LOW()              SPI_IMU_RST_GPIO_Port->BSRR = (uint32_t)SPI_IMU_RST_Pin << 16U
#define IMU_RST_SET_HIGH()             SPI_IMU_RST_GPIO_Port->BSRR = (uint32_t)SPI_IMU_RST_Pin
#define IMU_WAKE_PS0_SET_LOW()         SPI_IMU_PS0_WAKE_GPIO_Port->BSRR = (uint32_t)SPI_IMU_PS0_WAKE_Pin << 16U
#define IMU_WAKE_PS0_SET_HIGH()        SPI_IMU_PS0_WAKE_GPIO_Port->BSRR = (uint32_t)SPI_IMU_PS0_WAKE_Pin
#define ARRAY_LEN(a)                   ((sizeof(a))/(sizeof(a[0])))

/* Public typedef  ----------------------------------------------------------*/
typedef enum {
  SPI_OK      = 0x00U,
  SPI_ERROR   = 0x01U,
  SPI_BUSY    = 0x02U,
} spi_status_t;

typedef enum {
  IMU_OK      = 0x0U,
  IMU_ERROR   = 0x1U
} imu_status_t;

typedef enum {
  SPI_INIT    = 0x0U,
  SPI_DUMMY   = 0x1U,
  SPI_IDLE    = 0x2U,
  SPI_RD_HDR  = 0x3U,
  SPI_RD_BODY = 0x4U,
  SPI_WRITE   = 0x5U
} spi_state_t;

typedef struct {
  __attribute__((aligned(32)))
  uint8_t     RxBuffer[SH2_HAL_MAX_TRANSFER_IN];
  uint32_t    RxBuffer_Len;
  __attribute__((aligned(32)))
  uint8_t     Dummy_TxBuffer[SH2_HAL_MAX_TRANSFER_OUT];
  __attribute__((aligned(32)))
  uint8_t     TxBuffer[SH2_HAL_MAX_TRANSFER_OUT];
  uint32_t    TxBuffer_Len;
  spi_state_t state;
  uint32_t    rx_timestamp;
} sh2_spi_s;

/* Public variables ---------------------------------------------------------*/
extern volatile sh2_spi_s imu_spi;


/* Public function prototypes -----------------------------------------------*/
void imu_spi_activate(void);
void imu_spi_completed(void);
imu_status_t imu_sys_init(void);
imu_status_t imu_service(void);
#endif /* INC_SPI_DRIVER_H_ */
