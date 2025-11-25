/**
 * @file uart_comm.h
 * @brief UART communication for ESP-to-ESP protocol
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize UART for ESP-to-ESP communication
 */
esp_err_t uart_comm_init(void);

/**
 * @brief Send data via UART
 */
void uart_comm_send(const uint8_t *data, uint16_t length);

/**
 * @brief Start UART communication task
 */
void uart_comm_start_task(void);

#endif // UART_COMM_H