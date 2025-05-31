/*
 * retarget.c
 *
 * Author: Bowen
 */

#include "main.h"

uint8_t tx_ring_buf[PRINTF_TX_BUFFER_SIZE] = {0};
volatile uint16_t tx_head = 0;
volatile uint16_t tx_tail = 0;
volatile uint8_t tx_busy = 0;
extern UART_HandleTypeDef huart3;


/**
  * @brief  Enable use of C lib printf debug through UART3 ring buffer non-blocking
  */
int _write(int file, char *ptr, int len)
{
  (void)file;
  for (int i = 0; i < len; i++) {
    uint16_t next = (tx_head + 1) % PRINTF_TX_BUFFER_SIZE;
    while (next == tx_tail); // Wait if buffer full
    tx_ring_buf[tx_head] = ptr[i];
    tx_head = next;
  }

  if (!tx_busy) {
    tx_busy = 1;
    uint8_t byte = tx_ring_buf[tx_tail];
    tx_tail = (tx_tail + 1) % PRINTF_TX_BUFFER_SIZE;
    HAL_UART_Transmit_IT(&huart3, &byte, 1);
  }

  return len;
}
