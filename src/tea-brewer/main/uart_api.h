#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART on TX=GPIO43, RX=GPIO44.
 */
void uart_api_init(void);

/**
 * @brief Send a null-terminated string over UART.
 *
 * @param data Pointer to C-string to send.
 *
 * @return ESP_OK on success, error code otherwise.
 */
int uart_api_send(const char *data);

/**
 * @brief Receive data from UART into a buffer.
 *
 * This is a blocking read for up to @p timeout_ms milliseconds.
 *
 * @param buffer Destination buffer.
 * @param max_len Size of @p buffer in bytes.
 * @param timeout_ms Timeout in milliseconds.
 *
 * @return Number of bytes read (>=0) on success, negative error code on failure.
 */
int uart_api_receive(uint8_t *buffer, size_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
