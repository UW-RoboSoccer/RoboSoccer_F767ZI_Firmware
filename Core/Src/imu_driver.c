/**
  ******************************************************************************
  * @file       imu_driver.c
  * @brief      This file provides driver code for the BNO085 IMU
  *
  * @Reference  "1000-3625 SH-2 Reference Manual Rev1.9" [1]
  *             "HillcrestLabs BNO08X Data Sheet Rev1.17" [2]
  *             "1000-3535 Sensor Hub Transport Protocol Rev1.10" [3]
  *             "Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085" [4]
  *
  * @Note       1. The BNO08X supports 4 wire mode and implements SPI mode 3:
  *                CPOL = 1 (High) and CPHA = 1 (2 Edge); [2]
  *             2. Interrupt service routine in "spi.h", "gpio.h"
  *             3. Finite state machine spi_state_t [3] section 3.4.1
  *             4. PIN P0/PS0 should be connected to +3.3V via jumper
  *
  ******************************************************************************
  *
  * Author:    Bowen Zheng
  */

#include "spi.h"
#include "gpio.h"
#include "tim.h"
#include "imu_driver.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_util.h"
#include "shtp.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>


/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart2;
static sh2_Hal_t imu_hal;

volatile sh2_spi_s imu_spi = {
    .RxBuffer = {0},
    .Dummy_TxBuffer = {0},
    .TxBuffer = {0},
    .RxBuffer_Len = 0,
    .TxBuffer_Len = 0,
    .rx_timestamp = 0,
    .state = SPI_INIT
};

volatile uint8_t imu_hal_flags = 0;
volatile uint16_t imu_error_flags = 0;

volatile uint32_t debugCount0 = 0;
volatile uint32_t debugCount1 = 0;
volatile uint32_t debugCount2 = 0;


/*----------------- IMU Sensor Data ------------------*/
sh2_ProductIds_t prodIds;
volatile sh2_SensorValue_t imu_acc;
volatile sh2_SensorValue_t imu_gyro;
volatile sh2_SensorValue_t imu_grav;
/*----------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void spi_dummy_TransmitReceive(void);
static uint16_t imu_get_payload_length(void);
static void imu_read_shtp_pkt(uint16_t rx_len);
static void imu_hal_enable_interrupt(void);
static void imu_hal_disable_interrupt(void);
static void imu_hal_init_hardware(void);
static imu_status_t imu_reset_delay(uint32_t us_delay);
static void delay(uint32_t us_delay);
static spi_status_t imu_start_Receive(uint8_t *rx_buffer, uint16_t len);
static spi_status_t imu_start_Transmit(uint8_t *tx_buffer, uint16_t len);
static sh2_Hal_t *imu_get_hal(void);
static void imu_dataHandler(void *cookie, sh2_SensorEvent_t *pEvent);
static void sh2_eventHanlder(void *cookie, sh2_AsyncEvent_t *pEvt);
static void imu_report_sensorIds(void);
static imu_status_t imu_start_reports(void);


/* Function definitions -------------------------------------------------------*/
static void imu_hal_init_hardware(void)
{
  imu_hal_init_timer();
  imu_hal_init_gpio();
  imu_hal_init_spi();
}

static void imu_hal_enable_interrupt(void)
{
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

static void imu_hal_disable_interrupt(void)
{
  HAL_NVIC_DisableIRQ(SPI3_IRQn);
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
}


/**
  * @brief  MCU busy wait fn for INT pin to be serviced after imu reset
  * @param  us_delay microsecond delay
  * @note   "After power up or reset the BNO08X will assert the interrupt (HOST_INTN)
  *         indicating that the reset routine has completed and that the BNO08X
  *         is ready for communication." [2] section 5.2.1
  *         IMU_IN_RESET flag reset when INT pin is serviced
  */
static imu_status_t imu_reset_delay(uint32_t us_delay)
{
  volatile uint32_t now = MICRO_SECOND_STAMP();
  uint32_t start = now;
  while ((now - start) < us_delay) {
    now = MICRO_SECOND_STAMP();
    if (!(imu_hal_flags & IMU_IN_RESET)) {
      return IMU_OK;
    }
  }

  // boot failed, INT pin never serviced
  imu_error_flags |= IMU_FAILED_TO_BOOT;
  return IMU_ERROR;
}


static void delay(uint32_t us_delay)
{
  volatile uint32_t now = MICRO_SECOND_STAMP();
  uint32_t start = now;
  while ((now - start) < us_delay) {
      now = MICRO_SECOND_STAMP();
  }
}

/**
  * @brief  imu spi dummy blocking communication
  * @note   We need to establish SCLK in proper initial state.
  *         Do one SPI operation with reset asserted and no CS asserted to get clock sorted.
  */
static void spi_dummy_TransmitReceive(void)
{
  uint8_t dummyTx[1];
  uint8_t dummyRx[1];

  memset(dummyTx, 0xAA, sizeof(dummyTx));
  HAL_SPI_TransmitReceive(&hspi3, dummyTx, dummyRx, sizeof(dummyTx), 2);
}


/**
  * @brief  imu spi interrupt start receive helper fn, transmit dummy buffer
  * @param  *rx_buffer Pointer to receive buffer
  * @param  len number of bytes will process
  */
static spi_status_t imu_start_Receive(uint8_t *rx_buffer, uint16_t len)
{
  HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive_IT(&hspi3,
                                                        (uint8_t*)imu_spi.Dummy_TxBuffer,
                                                        rx_buffer,
                                                        len);
  switch(retval) {
    case HAL_OK:
      return SPI_OK;
    case HAL_BUSY:
      return SPI_BUSY;
    default:
      return SPI_ERROR;
  }
}


/**
  * @brief  imu spi interrupt start transmit helper fn, capture return data to RxBuffer
  * @param  *rx_buffer Pointer to receive buffer
  * @param  len number of bytes will process
  */
static spi_status_t imu_start_Transmit(uint8_t *tx_buffer, uint16_t len)
{
  HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive_IT(&hspi3,
                                                        tx_buffer,
                                                        (uint8_t*)imu_spi.RxBuffer,
                                                        len);
  switch(retval) {
    case HAL_OK:
      return SPI_OK;
    case HAL_BUSY:
      return SPI_BUSY;
    default:
      return SPI_ERROR;
  }
}


/**
  * @brief Read payload length from shtp header helper fn
  * @retval 16 bit payload length
  */
static uint16_t imu_get_payload_length(void)
{
  uint8_t shtp_len_lsb = imu_spi.RxBuffer[0];
  uint8_t shtp_len_msb = imu_spi.RxBuffer[1];
  uint16_t shtp_len_full = ((shtp_len_msb << 8U) | shtp_len_lsb);

  /*
   *  Check for a failed IMU peripheral
   *  "A length of 65535 (0xFFFF) is reserved because a failed peripheral
   *  can too easily produce 0xFFFF".[3] section 2.2.1
   */
  if (shtp_len_full == 0xFFFFU) {
    imu_error_flags |= IMU_PERIPHERAL_FAILURE;
    return 0;
  }

  /* - Required Linear acc + angular vel + gravity vector total 16B << 1024B
     - Hence no need to handle bit 15 (CONT bit)
     - Packet length from bit 0 -> bit 14 - header size
   */
  shtp_len_full &= SHTP_PKT_LEN_MSK;
  return shtp_len_full;
}


/**
  * @brief  Non-blocking read shtp packet helper fn
  * @param  pkt_len packet length word
  * @retval Is spi transmission successful
  */
static void imu_read_shtp_pkt(uint16_t rx_len)
{
  if (imu_start_Receive((uint8_t*)imu_spi.RxBuffer + SHTP_HEADER_SIZE,
                        rx_len - SHTP_HEADER_SIZE) == SPI_ERROR) {
    imu_error_flags |= IMU_READ_PKT_FAILURE;
  }

  return;
}


/**
  * @brief Attempt to start a spi operation
  */
void imu_spi_activate(void)
{
  // Return if bus is busy or previous packet still exist
  if (imu_spi.state != SPI_IDLE || imu_spi.RxBuffer_Len > 0) {
    return;
  }

  // INT is serviced
  if (imu_hal_flags & IMU_INT_SERVICED) {
    imu_hal_flags &= ~IMU_INT_SERVICED;

    /*
     * End IMU host-hub handshakeing per [3] section 3.1
     */
    IMU_CS_SET_LOW();
    // Transmit if TxBuffer exist
    if (imu_spi.TxBuffer_Len > 0) {
      imu_spi.state = SPI_WRITE;
      spi_status_t tx_retval = imu_start_Transmit((uint8_t*)imu_spi.TxBuffer, imu_spi.TxBuffer_Len);
      IMU_WAKE_PS0_SET_HIGH();

      if (tx_retval == SPI_ERROR) {
        imu_error_flags |= IMU_TRANSMIT_ERROR;
        imu_spi.state = SPI_IDLE;
        IMU_CS_SET_HIGH();
        debugCount0 += 1;
      } else if (tx_retval == SPI_BUSY) {
         //Retry when next INT pin is serviced
        imu_spi.state = SPI_IDLE;
        IMU_CS_SET_HIGH();
        debugCount1 += 1;
      }
    } else {
      // Read header
      imu_spi.state = SPI_RD_HDR;
      spi_status_t rx_retval = imu_start_Receive((uint8_t*)imu_spi.RxBuffer, SHTP_HEADER_SIZE);

      if (rx_retval == SPI_ERROR) {
        imu_error_flags |= IMU_READ_HEADER_FAILURE;
        IMU_CS_SET_HIGH();
      } else if (rx_retval == SPI_BUSY) {
        //Retry when next INT pin is serviced
        IMU_CS_SET_HIGH();
      }
    }

  }

  return;
}


/**
  * @brief Handles the end of a spi operation based on SPI state
  * @note  Called in HAL_SPI_TxRxCpltCallback (spi transfer completed)
  */
void imu_spi_completed(void)
{
  // Get avaliable payload length, truncate if too long
  uint16_t rx_len = imu_get_payload_length();
  if (rx_len > sizeof(imu_spi.RxBuffer)) {
    rx_len = sizeof(imu_spi.RxBuffer);
  }
  if (rx_len < SHTP_HEADER_SIZE) {
    rx_len = SHTP_HEADER_SIZE;
    imu_error_flags |= IMU_HEADER_DATA_ERROR;
  }

  switch (imu_spi.state) {
    case SPI_DUMMY:
      imu_spi.state = SPI_IDLE;
      break;

    case SPI_RD_HDR:
      // If sensor data exist, branch to read body
      if (rx_len > SHTP_HEADER_SIZE) {
        imu_spi.state = SPI_RD_BODY;
        imu_read_shtp_pkt(rx_len);
      } else {
        // Finished reading header, reset buf length, chip select, branch to idle
        IMU_CS_SET_HIGH();
        imu_spi.RxBuffer_Len = 0;
        imu_spi.state = SPI_IDLE;
        imu_spi_activate();
      }
      break;

    case SPI_RD_BODY:
      // Finished reading packet, branch to idle
      IMU_CS_SET_HIGH();
      imu_spi.RxBuffer_Len = rx_len;
      imu_spi.state = SPI_IDLE;
      imu_spi_activate();
      break;

    case SPI_WRITE:
      // Finished reading packet, reset buf length, chip select, branch to idle
      IMU_CS_SET_HIGH();
      // After a write operation, rx length cannot be larger than tx length
      if (rx_len > imu_spi.TxBuffer_Len) {
        imu_spi.RxBuffer_Len = imu_spi.TxBuffer_Len;
      } else {
        imu_spi.RxBuffer_Len = rx_len;
      }
      imu_spi.TxBuffer_Len = 0;
      imu_spi.state = SPI_IDLE;
      break;

    default:
      // Should never reach here
      imu_spi.state = SPI_IDLE;
      IMU_CS_SET_HIGH();
      imu_error_flags |= IMU_SPI_COMPLETE_FSM_ERROR;
      break;
  }

  return;
}


/**
  * @brief  IMU SH2 fn pointer for sh2_Hal_s (*open)
  * @note   The SH2 interface uses these functions to access the underlying
  *         communications device. Detailed description can be found in "sh2_hal.h"
  */
static int imu_hal_open(sh2_Hal_t *self)
{
  (void)self;

  if(imu_hal_flags & IMU_HAL_OPENED) {
    return SH2_ERR;
  }
  imu_hal_flags |= IMU_HAL_OPENED;

  imu_hal_init_hardware();

  /*
   * Begin startup timing
   * See [2] section 6.5.3, 6.5.4 for timing diagram
   */
  IMU_RST_SET_LOW();
  // In reset flag will be reseted when INT pin is pulled low.
  imu_hal_flags |= IMU_IN_RESET;
  // Deassert chip select pin
  IMU_CS_SET_HIGH();
  imu_spi.RxBuffer_Len = 0;
  imu_spi.TxBuffer_Len = 0;
  imu_hal_flags &= ~IMU_INT_SERVICED;

  // Perform blocking dummy operation
  imu_spi.state = SPI_DUMMY;
  spi_dummy_TransmitReceive();
  imu_spi.state = SPI_IDLE;
  delay(IMU_RESET_DELAY);

  // To boot in SHTP-SPI mode, must have PS1=1, PS0=1 while imu in reset.
  // PS1 will be set via 3.3v jumper AND PS0_WAKEN sig is set high.
  // PS0 will be repurposed to wake signal after reset
  IMU_WAKE_PS0_SET_HIGH();
  // Leave reset mode
  IMU_RST_SET_HIGH();

  // Start imu internal initialization
  // Stop the interrupt service routine as INT not reliable at this moment
  imu_hal_disable_interrupt();
  delay(IMU_START_INIT_DELAY);
  imu_hal_enable_interrupt();

  // Wait for INT pin to be asserted, failed to boot if timed out
  if (imu_reset_delay(IMU_START_RESET_DELAY) != IMU_OK) {
    printf("! IMU FAILURE TO BOOT \n\r");
    return SH2_ERR_IO;
  }
  /*
   * End startup timing
   */

  printf("IMU BOOT SUCCESS \n\r");
  return SH2_OK;
}


/**
  * @brief  IMU SH2 fn pointers for sh2_Hal_s (*close)
  * @note   The SH2 interface uses these functions to access the underlying
  *         communications device. Detailed description can be found in "sh2_hal.h"
  */
static void imu_hal_close(sh2_Hal_t *self)
{
  (void)self;
  imu_hal_disable_interrupt();

  imu_spi.state = SPI_INIT;

  // Hold imu in reset
  IMU_RST_SET_LOW();
  IMU_CS_SET_HIGH();

  // Deinit peripherals
  HAL_SPI_DeInit(&hspi3);
  __HAL_TIM_DISABLE(&htim2);

  // close sh2
  imu_hal_flags &= ~IMU_HAL_OPENED;
}


/**
  * @brief  IMU SH2 fn pointer for sh2_Hal_s (*read)
  * @note   The SH2 interface uses these functions to access the underlying
  *         communications device. Detailed description can be found in "sh2_hal.h"
  */
static int imu_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
  (void)self;
  int retval = 0;

  // If there is received data available
  if (imu_spi.RxBuffer_Len > 0) {
    // And if the data will fit
    if (len >= imu_spi.RxBuffer_Len) {
      // Copy data to the client buffer
      memcpy(pBuffer, (uint8_t*)imu_spi.RxBuffer, (uint32_t)imu_spi.RxBuffer_Len);
      retval = imu_spi.RxBuffer_Len;
      // Set timestamp
      *t_us = imu_spi.rx_timestamp;
      // Clear buffer len
      imu_spi.RxBuffer_Len = 0;
    } else {
      // Discard what was read and return error because buffer was too small.
      retval = SH2_ERR_BAD_PARAM;
      imu_spi.RxBuffer_Len = 0;
    }

    // Now that rxBuf is empty, activate SPI to check write task
    imu_hal_disable_interrupt();
    imu_spi_activate();
    imu_hal_enable_interrupt();
  }

  return retval;
}


/**
  * @brief  IMU SH2 fn pointer for sh2_Hal_s (*write)
  * @note   The SH2 interface uses these functions to access the underlying
  *         communications device. Detailed description can be found in "sh2_hal.h"
  */
static int imu_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
  int retval = SH2_OK;

  // Assert parameters
  if ((self == 0) || (len > sizeof(imu_spi.TxBuffer)) || ((len > 0) && (pBuffer == 0))) {
    return SH2_ERR_BAD_PARAM;
  }

  // If tx is occupied, return 0 to try again later
  if (imu_spi.TxBuffer_Len != 0) {
    IMU_WAKE_PS0_SET_HIGH();
    return 0;
  }

  // Copy data to tx buffer
  memcpy((uint8_t*)imu_spi.TxBuffer, pBuffer, len);
  imu_spi.TxBuffer_Len = len;
  retval = len;

  /*
   * Start IMU host-hub handshakeing per [3] section 3.1
   * Start IMU wake up
   */
  imu_hal_disable_interrupt();
  IMU_WAKE_PS0_SET_LOW();
  imu_hal_enable_interrupt();

  return retval;
}


/**
  * @brief  IMU SH2 fn pointer for sh2_Hal_s (*getTimeUs)
  * @note   The SH2 interface uses these functions to access the underlying
  *         communications device. Detailed description can be found in "sh2_hal.h"
  */
static uint32_t imu_hal_get_time(sh2_Hal_t *self)
{
  (void)self;
  return MICRO_SECOND_STAMP();
}


/**
  * @brief  IMU interface to SH-2 middleware
  */
static sh2_Hal_t *imu_get_hal(void)
{
  // Assign fn addresses
  imu_hal.open = imu_hal_open;
  imu_hal.close = imu_hal_close;
  imu_hal.read = imu_hal_read;
  imu_hal.write = imu_hal_write;
  imu_hal.getTimeUs = imu_hal_get_time;

  return &imu_hal;
}


/**
  * @brief  sh2 non-sensor event callback handler definition
  */
static void sh2_eventHanlder(void *cookie, sh2_AsyncEvent_t *pEvt)
{
  (void)cookie;
  switch (pEvt->eventId) {
    // If reset occured, set a flag to reconfigure sensor
    case SH2_RESET:
      imu_hal_flags |= IMU_RESET_OCCURRED;
      printf("sensor reset occurred \n\r");
      break;

    case SH2_GET_FEATURE_RESP:
      printf("sensor 0x%02X accepted %lu us (%.1f Hz)\n\r",
             pEvt->sh2SensorConfigResp.sensorId,
             pEvt->sh2SensorConfigResp.sensorConfig.reportInterval_us,
             1e6 / (float)pEvt->sh2SensorConfigResp.sensorConfig.reportInterval_us);
      break;

    case SH2_SHTP_EVENT: {
      // create an array of 8 pointers to event names
      static const char *const shtp_event_name[] = {
          "TX_DISCARD",
          "SHORT_FRAGMENT",
          "TOO_LARGE_PAYLOADS",
          "BAD_RX_CHAN",
          "BAD_TX_CHAN",
          "BAD_FRAGMENT",
          "BAD_SN",
          "INTERRUPTED_PAYLOAD"
      };
      uint8_t arr_max =  sizeof(shtp_event_name) / sizeof(shtp_event_name[1]);
      if (pEvt->shtpEvent < arr_max) {
        printf("[shtp event] %s (%u) \n\r",
              shtp_event_name[pEvt->shtpEvent],
              pEvt->shtpEvent);
      } else {
        printf("[shtp event] UNKNOWN (%u)\n\r", pEvt->shtpEvent);
      }
      break;
    }
  }
}



/**
  * @brief  IMU sensor data event callback handler definition
  */
static void imu_dataHandler(void * cookie, sh2_SensorEvent_t *pEvent)
{
  (void)cookie;
  sh2_SensorValue_t value;

  if (sh2_decodeSensorEvent(&value, pEvent) != SH2_OK) {
    imu_error_flags |= IMU_DECODE_SENSOR_EVENT_ERROR;
    return;
  }

  switch (value.sensorId) {
    case SH2_LINEAR_ACCELERATION:
      imu_acc = value;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      imu_gyro = value;
      break;
    case SH2_GRAVITY:
      imu_grav = value;
      break;
    default:
      break;
  }
}


/**
  * @brief Read product ids with version info from the hub.
  */
static void imu_report_sensorIds(void)
{
  int status;

  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);

  if (status < 0) {
    printf("Error from sh2_getProdIds.\n\r");
    return;
  }

  // Report the results
  for (int n = 0; n < prodIds.numEntries; n++) {
    printf("IMU Part %ld : Version %d.%d.%d Build %ld Reset Cause %d \n\r",
           prodIds.entry[n].swPartNumber,
           prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor,
           prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber,
           prodIds.entry[n].resetCause);
  }
}


/**
  * @brief Configure imu to generate periodic reports
  * @note Each entry of sensorConfig[] represents one sensor to be configured in the loop
  * @retval Is imu operation successful
  */
static imu_status_t imu_start_reports(void)
{
  static struct {
    int                 sensorId;
    sh2_SensorConfig_t  config;
  } sensorConfig[] = {
      // Enable sensors
      { SH2_LINEAR_ACCELERATION,  {.reportInterval_us = BNO08X_REPORT_400HZ,
                                   .alwaysOnEnabled   = 1} },
      { SH2_GYROSCOPE_CALIBRATED, {.reportInterval_us = BNO08X_REPORT_400HZ,
                                   .alwaysOnEnabled   = 1} },
      { SH2_GRAVITY,              {.reportInterval_us = BNO08X_REPORT_400HZ,
                                   .alwaysOnEnabled   = 1} },

      /*
       * Disable sensors
       * "Sensor operating rate is controlled through the report interval field.
       * When set to zero the sensor is off." [1] section 5.4.1
       */
      { SH2_RAW_MAGNETOMETER,            {.reportInterval_us = 0} },
      { SH2_MAGNETIC_FIELD_CALIBRATED,   {.reportInterval_us = 0} },
      { SH2_MAGNETIC_FIELD_UNCALIBRATED, {.reportInterval_us = 0} },
      { SH2_GEOMAGNETIC_ROTATION_VECTOR, {.reportInterval_us = 0} },
      { SH2_ROTATION_VECTOR,             {.reportInterval_us = 0} },
      { SH2_ARVR_STABILIZED_RV,          {.reportInterval_us = 0} },
  };

  for (uint8_t n = 0; n < ARRAY_LEN(sensorConfig); n++) {
    int sensorId = sensorConfig[n].sensorId;
    if (sh2_setSensorConfig(sensorId, &sensorConfig[n].config) != SH2_OK) {
      if (sensorConfig[n].config.reportInterval_us != 0) {
        printf("Error while enabling sensor %d\n\r", sensorId);
        imu_error_flags |= IMU_SENSOR_ENABLE_FAILURE;
        return IMU_ERROR;
      } else {
        printf("Error while disabling sensor %d\n\r", sensorId);
        imu_error_flags |= IMU_SENSOR_DISABLE_FAILURE;
        return IMU_ERROR;
      }
    }
  }

  return IMU_OK;
}


/**
  * @brief  IMU initialzation fn
  * @retval Is imu operation successful
  */
imu_status_t imu_sys_init(void)
{
  // Open SH2 interface and register SH2 events
  if (sh2_open(imu_get_hal(), sh2_eventHanlder, NULL) != SH2_OK) {
    imu_error_flags |= IMU_SH2_FAILED_TO_OPEN;
    return IMU_ERROR;
  }


  // Register sensor data events
  sh2_setSensorCallback(imu_dataHandler, NULL);
  imu_status_t retval = imu_start_reports();

  // Get product ids
  imu_report_sensorIds();

  // Reset imu reseted flag
  imu_hal_flags &= ~IMU_RESET_OCCURRED;

  // Start sensor reports
  return retval;
}


/**
 * @brief  IMU service helper fn
 * @note   Must be called at freq > BNO085 report interval to keep data flowing.
 * @retval Is imu operation successful
 */
imu_status_t imu_service(void)
{
  // Reconfigure the imu if reset occured
  if (imu_hal_flags & IMU_RESET_OCCURRED) {
    if (imu_start_reports() != IMU_OK) {
      return IMU_ERROR;
    }
    imu_hal_flags &= ~IMU_RESET_OCCURRED;
  }

  sh2_service();
  return IMU_OK;
}
