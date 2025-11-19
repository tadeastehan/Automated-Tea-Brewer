#include "uart_api.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#define UART_PORT       UART_NUM_1
#define UART_TX_GPIO    (43)
#define UART_RX_GPIO    (44)
#define UART_BAUD_RATE  (115200)

void uart_api_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configure UART parameters
    uart_param_config(UART_PORT, &uart_config);

    // Set UART pins (TX, RX, RTS, CTS)
    uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver with RX buffer and no TX buffer (TX uses blocking writes)
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
}

int uart_api_send(const char *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(data);
    if (len == 0) {
        return ESP_OK;
    }

    int written = uart_write_bytes(UART_PORT, data, len);
    return (written < 0) ? written : ESP_OK;
}

int uart_api_receive(uint8_t *buffer, size_t max_len, uint32_t timeout_ms)
{
    if (buffer == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int len = uart_read_bytes(UART_PORT,
                              buffer,
                              max_len,
                              pdMS_TO_TICKS(timeout_ms));

    return len;
}
