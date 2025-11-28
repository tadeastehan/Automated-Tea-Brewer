/**
 * @file uart_comm.c
 * @brief UART communication implementation
 */

#include "uart_comm.h"
#include "../main_pins.h"
#include "../protocol/protocol.h"
#include "../motor/motor_control.h"
#include "../temperature_sensor/thermometer.h"
#include "../pot_sensor/pot_sensor.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "UART_COMM";

#define UART_NUM            UART_NUM_0
#define UART_BAUD           115200
#define UART_BUF_SIZE       256

/* Communication watchdog constants */
#define COMM_WATCHDOG_TIMEOUT_MS    30000   // 30 seconds without communication = disconnected
#define COMM_CONNECTION_TIMEOUT_MS  10000   // 10 seconds to establish connection

/* Response buffer */
static uint8_t tx_buffer[64];

/* Communication watchdog state */
static int64_t last_command_time = 0;
static bool induction_is_on = false;
static bool esp1_connected = false;     // Track if ESP #1 is connected

/* ============================================
   INITIALIZATION
   ============================================ */
esp_err_t uart_comm_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, PIN_UART_TX, PIN_UART_RX, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized (TX:%d, RX:%d, %d baud)", 
             PIN_UART_TX, PIN_UART_RX, UART_BAUD);
    
    return ESP_OK;
}

void uart_comm_send(const uint8_t *data, uint16_t length)
{
    uart_write_bytes(UART_NUM, data, length);
}

/* ============================================
   RESPONSE HELPERS
   ============================================ */
static void send_ack(void)
{
    uint8_t len = proto_build_ack(tx_buffer);
    uart_comm_send(tx_buffer, len);
}

static void send_nak(protocol_error_t error)
{
    uint8_t len = proto_build_nak(tx_buffer, error);
    uart_comm_send(tx_buffer, len);
}

static void send_status(void)
{
    motor_status_t status;
    motor_get_status(&status);
    uint8_t len = proto_build_status(tx_buffer, &status);
    uart_comm_send(tx_buffer, len);
}

static void send_position(void)
{
    int32_t steps = motor_get_position_steps();
    float percent = motor_get_position_percent();
    uint8_t len = proto_build_position(tx_buffer, steps, percent);
    uart_comm_send(tx_buffer, len);
}

static void send_sgt(void)
{
    int8_t sgt = motor_get_sgt();
    uint8_t len = proto_build_sgt(tx_buffer, sgt);
    uart_comm_send(tx_buffer, len);
}

static void send_pong(void)
{
    uint8_t len = proto_build_pong(tx_buffer);
    uart_comm_send(tx_buffer, len);
}

static void send_notify(uint8_t type, uint8_t data)
{
    uint8_t len = proto_build_notify(tx_buffer, type, data);
    uart_comm_send(tx_buffer, len);
}

static void send_temperature(void)
{
    float object_temp = 0.0f;
    float ambient_temp = 0.0f;
    
    if (thermometer_is_initialized()) {
        thermometer_get_object_temp(&object_temp);
        thermometer_get_ambient_temp(&ambient_temp);
    }
    
    uint8_t len = proto_build_temperature(tx_buffer, object_temp, ambient_temp);
    uart_comm_send(tx_buffer, len);
}

static void send_pot_presence(void)
{
    bool is_present = false;
    uint16_t distance_mm = 0;
    
    if (pot_sensor_is_initialized()) {
        pot_sensor_get_distance(&distance_mm);
        is_present = pot_sensor_is_present();
    }
    
    uint8_t len = proto_build_pot_presence(tx_buffer, is_present, distance_mm);
    uart_comm_send(tx_buffer, len);
}

/* ============================================
   INDUCTION COOKER CONTROL
   ============================================ */
static bool induction_initialized = false;

static void induction_init(void)
{
    if (!induction_initialized) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << PIN_INDUCTION),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(PIN_INDUCTION, 0);  // Start with induction OFF
        induction_initialized = true;
        ESP_LOGI(TAG, "Induction cooker GPIO initialized (pin %d)", PIN_INDUCTION);
    }
}

static void induction_on(void)
{
    induction_init();
    gpio_set_level(PIN_INDUCTION, 1);
    induction_is_on = true;
    ESP_LOGI(TAG, "Induction cooker ON");
}

static void induction_off(void)
{
    induction_init();
    gpio_set_level(PIN_INDUCTION, 0);
    induction_is_on = false;
    ESP_LOGI(TAG, "Induction cooker OFF");
}

/* ============================================
   COMMAND HANDLER
   ============================================ */
static void handle_command(proto_frame_t *frame)
{
    esp_err_t ret;
    
    /* Update last command time and connection status for watchdog */
    last_command_time = esp_timer_get_time();
    if (!esp1_connected) {
        esp1_connected = true;
        ESP_LOGI(TAG, "ESP #1 connected");
    }
    
    ESP_LOGD(TAG, "Received command: 0x%02X, data_len: %d", frame->cmd, frame->length);
    
    switch (frame->cmd) {
        case CMD_PING:
            ESP_LOGI(TAG, "PING received");
            send_pong();
            break;
            
        case CMD_GET_STATUS:
            ESP_LOGD(TAG, "GET_STATUS");
            send_status();
            break;
            
        case CMD_CALIBRATE:
            ESP_LOGI(TAG, "CALIBRATE command");
            send_ack();  /* Acknowledge command received */
            ret = motor_calibrate();
            if (ret == ESP_OK) {
                send_notify(NOTIFY_CALIBRATE_DONE, 0);
            } else {
                send_notify(NOTIFY_ERROR, PROTO_ERR_MOTOR_FAULT);
            }
            break;
            
        case CMD_HOME:
            ESP_LOGI(TAG, "HOME command");
            ret = motor_home();
            if (ret == ESP_OK) {
                send_ack();
                send_notify(NOTIFY_HOME_COMPLETE, 0);
            } else {
                send_nak(PROTO_ERR_NOT_CALIBRATED);
            }
            break;
            
        case CMD_MOVE_PERCENT:
            if (frame->length >= 2) {
                int16_t pct_fixed;
                memcpy(&pct_fixed, frame->data, 2);
                float percent = (float)pct_fixed / 10.0f;
                
                ESP_LOGI(TAG, "MOVE_PERCENT: %.1f%%", percent);
                
                ret = motor_move_to_percent(percent);
                if (ret == ESP_OK) {
                    send_ack();
                    /* Position update sent after move completes */
                    send_position();
                    send_notify(NOTIFY_MOVE_COMPLETE, 0);
                } else if (ret == ESP_ERR_INVALID_STATE) {
                    motor_status_t status;
                    motor_get_status(&status);
                    if (!status.is_homed) {
                        send_nak(PROTO_ERR_NOT_HOMED);
                    } else {
                        send_nak(PROTO_ERR_NOT_CALIBRATED);
                    }
                } else {
                    send_nak(PROTO_ERR_MOTOR_FAULT);
                }
            } else {
                send_nak(PROTO_ERR_INVALID_PARAM);
            }
            break;
            
        case CMD_MOVE_POSITION:
            if (frame->length >= 4) {
                int32_t position;
                memcpy(&position, frame->data, 4);
                
                ESP_LOGI(TAG, "MOVE_POSITION: %ld", (long)position);
                
                ret = motor_move_to_position(position);
                if (ret == ESP_OK) {
                    send_ack();
                    send_position();
                    send_notify(NOTIFY_MOVE_COMPLETE, 0);
                } else {
                    send_nak(PROTO_ERR_NOT_HOMED);
                }
            } else {
                send_nak(PROTO_ERR_INVALID_PARAM);
            }
            break;
            
        case CMD_STOP:
            ESP_LOGI(TAG, "STOP command");
            motor_stop();
            send_ack();
            break;
            
        case CMD_ENABLE:
            ESP_LOGI(TAG, "ENABLE command");
            motor_enable();
            send_ack();
            break;
            
        case CMD_DISABLE:
            ESP_LOGI(TAG, "DISABLE command");
            motor_disable();
            send_ack();
            break;
            
        case CMD_SET_SGT:
            if (frame->length >= 1) {
                int8_t sgt = (int8_t)frame->data[0];
                ESP_LOGI(TAG, "SET_SGT: %d", sgt);
                motor_set_sgt(sgt);
                send_ack();
            } else {
                send_nak(PROTO_ERR_INVALID_PARAM);
            }
            break;
            
        case CMD_GET_SGT:
            ESP_LOGD(TAG, "GET_SGT");
            send_sgt();
            break;
            
        case CMD_SAVE_CALIBRATION:
            ESP_LOGI(TAG, "SAVE_CALIBRATION command");
            ret = motor_save_calibration();
            if (ret == ESP_OK) {
                send_ack();
            } else {
                send_nak(PROTO_ERR_MOTOR_FAULT);
            }
            break;
            
        case CMD_CLEAR_CALIBRATION:
            ESP_LOGI(TAG, "CLEAR_CALIBRATION command");
            motor_clear_calibration();
            send_ack();
            break;
            
        case CMD_GET_TEMPERATURE:
            ESP_LOGD(TAG, "GET_TEMPERATURE");
            send_temperature();
            break;
            
        case CMD_GET_POT_PRESENCE:
            ESP_LOGD(TAG, "GET_POT_PRESENCE");
            send_pot_presence();
            break;
            
        case CMD_INDUCTION_ON:
            ESP_LOGI(TAG, "INDUCTION_ON command");
            // Safety check: only allow induction ON if ESP #1 is connected
            if (!esp1_connected) {
                ESP_LOGW(TAG, "Rejecting INDUCTION_ON - ESP #1 not connected");
                send_nak(PROTO_ERR_INVALID_CMD);
            } else {
                // Check if motor is moving (safety interlock)
                motor_status_t motor_status;
                motor_get_status(&motor_status);
                if (motor_status.state == MOTOR_STATE_MOVING || 
                    motor_status.state == MOTOR_STATE_HOMING || 
                    motor_status.state == MOTOR_STATE_CALIBRATING) {
                    ESP_LOGW(TAG, "Rejecting INDUCTION_ON - motor is busy");
                    send_nak(PROTO_ERR_MOTOR_FAULT);
                } else {
                    induction_on();
                    send_ack();
                }
            }
            break;
            
        case CMD_INDUCTION_OFF:
            ESP_LOGI(TAG, "INDUCTION_OFF command");
            induction_off();
            send_ack();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", frame->cmd);
            send_nak(PROTO_ERR_INVALID_CMD);
            break;
    }
}

/* ============================================
   UART COMMUNICATION TASK
   ============================================ */
static void uart_comm_task(void *pvParameters)
{
    uint8_t rx_byte;
    proto_frame_t frame;
    
    ESP_LOGI(TAG, "UART communication task started");
    
    /* Reset protocol parser */
    proto_parser_reset();
    
    /* Initialize last command time */
    last_command_time = esp_timer_get_time();
    
    while (1) {
        /* Read one byte at a time (non-blocking with short timeout) */
        int len = uart_read_bytes(UART_NUM, &rx_byte, 1, pdMS_TO_TICKS(10));
        
        if (len > 0) {
            /* Feed byte to protocol parser */
            if (proto_parse_byte(rx_byte, &frame)) {
                if (frame.valid) {
                    ESP_LOGD(TAG, "Valid frame received, cmd=0x%02X", frame.cmd);
                    handle_command(&frame);
                } else {
                    ESP_LOGW(TAG, "Invalid frame (CRC error)");
                }
            }
        }
        
        /* Communication watchdog - monitor ESP #1 connection */
        int64_t now = esp_timer_get_time();
        int64_t elapsed_ms = (now - last_command_time) / 1000;
        
        /* Periodic debug logging every 5 seconds */
        static int64_t last_debug_log = 0;
        if ((now - last_debug_log) / 1000 > 5000) {
            last_debug_log = now;
            ESP_LOGI(TAG, "Watchdog: connected=%d, induction=%d, elapsed=%lld ms, timeout=%d ms",
                     esp1_connected, induction_is_on, (long long)elapsed_ms, COMM_WATCHDOG_TIMEOUT_MS);
        }
        
        /* Check for connection timeout */
        if (esp1_connected && elapsed_ms > COMM_WATCHDOG_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Communication watchdog triggered - ESP #1 disconnected!");
            ESP_LOGW(TAG, "Elapsed time: %lld ms (timeout: %d ms)", (long long)elapsed_ms, COMM_WATCHDOG_TIMEOUT_MS);
            esp1_connected = false;
            
            /* Turn off induction cooker for safety */
            if (induction_is_on) {
                ESP_LOGW(TAG, "Turning off induction cooker for safety");
                induction_off();
            }
        }
        
        /* Small yield to prevent watchdog issues */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void uart_comm_start_task(void)
{
    xTaskCreate(uart_comm_task, "uart_comm", 4096, NULL, 10, NULL);
}