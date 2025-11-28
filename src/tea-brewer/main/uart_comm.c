/**
 * @file uart_comm.c
 * @brief UART communication with Motor Controller (ESP #2)
 */

#include "uart_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "UART_COMM";

/* ============================================
   PROTOCOL CONSTANTS
   ============================================ */
#define PROTO_START_BYTE        0xAA
#define PROTO_MAX_DATA_SIZE     252

/* Command IDs */
#define CMD_PING                0x01
#define CMD_GET_STATUS          0x02
#define CMD_CALIBRATE           0x10
#define CMD_HOME                0x11
#define CMD_MOVE_PERCENT        0x20
#define CMD_MOVE_POSITION       0x21
#define CMD_STOP                0x22
#define CMD_ENABLE              0x23
#define CMD_DISABLE             0x24
#define CMD_SET_SGT             0x30
#define CMD_GET_SGT             0x31
#define CMD_SAVE_CALIBRATION    0x40
#define CMD_CLEAR_CALIBRATION   0x41
#define CMD_GET_TEMPERATURE     0x50
#define CMD_GET_POT_PRESENCE    0x51

/* Response IDs */
#define RSP_ACK                 0x80
#define RSP_NAK                 0x81
#define RSP_STATUS              0x82
#define RSP_POSITION            0x83
#define RSP_SGT                 0x84
#define RSP_PONG                0x85
#define RSP_TEMPERATURE         0x86
#define RSP_POT_PRESENCE        0x87

/* Notification IDs */
#define NOTIFY_MOVE_COMPLETE    0xA0
#define NOTIFY_HOME_COMPLETE    0xA1
#define NOTIFY_CALIBRATE_DONE   0xA2
#define NOTIFY_ERROR            0xAF

/* ============================================
   UART CONFIGURATION
   ============================================ */
#define UART_NUM                UART_NUM_1
#define UART_BUF_SIZE           256
#define PING_INTERVAL_MS        2000
#define PING_TIMEOUT_MS         30000

/* ============================================
   PRIVATE DATA
   ============================================ */
static TaskHandle_t uart_task_handle = NULL;
static SemaphoreHandle_t status_mutex = NULL;
static bool is_running = false;

/* Cached status */
static motor_status_t cached_status = {0};
static uint8_t last_error = 0;
static int64_t last_response_time = 0;

/* Callbacks */
static uart_comm_status_cb_t status_callback = NULL;
static uart_comm_move_complete_cb_t move_complete_callback = NULL;
static uart_comm_home_complete_cb_t home_complete_callback = NULL;
static uart_comm_calibrate_complete_cb_t calibrate_complete_callback = NULL;
static uart_comm_error_cb_t error_callback = NULL;
static uart_comm_temperature_cb_t temperature_callback = NULL;
static uart_comm_pot_presence_cb_t pot_presence_callback = NULL;

/* Cached temperature */
static float cached_object_temp = 0.0f;
static float cached_ambient_temp = 0.0f;

/* Cached pot presence */
static bool cached_pot_present = false;
static uint16_t cached_pot_distance_mm = 0;

/* TX buffer */
static uint8_t tx_buffer[64];

/* Parser state */
typedef enum {
    PARSE_WAIT_START,
    PARSE_LENGTH,
    PARSE_CMD,
    PARSE_DATA,
    PARSE_CRC
} parse_state_t;

static parse_state_t parse_state = PARSE_WAIT_START;
static uint8_t parse_length = 0;
static uint8_t parse_index = 0;
static uint8_t parse_buffer[PROTO_MAX_DATA_SIZE + 3];

/* Parsed frame structure */
typedef struct {
    uint8_t cmd;
    uint8_t length;
    uint8_t data[PROTO_MAX_DATA_SIZE];
    bool valid;
} proto_frame_t;

/* ============================================
   PROTOCOL HELPERS
   ============================================ */
static uint8_t calculate_crc(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

static uint8_t build_frame(uint8_t *buffer, uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    uint8_t idx = 0;
    
    buffer[idx++] = PROTO_START_BYTE;
    buffer[idx++] = data_len + 1;  /* Length = data + cmd */
    buffer[idx++] = cmd;
    
    if (data && data_len > 0) {
        memcpy(&buffer[idx], data, data_len);
        idx += data_len;
    }
    
    /* Calculate CRC over length, cmd, and data */
    buffer[idx] = calculate_crc(&buffer[1], data_len + 2);
    idx++;
    
    return idx;
}

static void send_command(uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    uint8_t frame_len = build_frame(tx_buffer, cmd, data, data_len);
    uart_write_bytes(UART_NUM, tx_buffer, frame_len);
    ESP_LOGD(TAG, "Sent cmd: 0x%02X, len: %d", cmd, frame_len);
}

/* ============================================
   PARSER
   ============================================ */
static void parser_reset(void)
{
    parse_state = PARSE_WAIT_START;
    parse_length = 0;
    parse_index = 0;
}

static bool parse_byte(uint8_t byte, proto_frame_t *frame)
{
    frame->valid = false;
    
    switch (parse_state) {
        case PARSE_WAIT_START:
            if (byte == PROTO_START_BYTE) {
                parse_state = PARSE_LENGTH;
                parse_index = 0;
            }
            break;
            
        case PARSE_LENGTH:
            parse_length = byte;
            parse_buffer[0] = byte;
            parse_index = 1;
            if (parse_length == 0 || parse_length > PROTO_MAX_DATA_SIZE + 1) {
                parser_reset();
            } else {
                parse_state = PARSE_CMD;
            }
            break;
            
        case PARSE_CMD:
            parse_buffer[parse_index++] = byte;
            if (parse_length == 1) {
                parse_state = PARSE_CRC;
            } else {
                parse_state = PARSE_DATA;
            }
            break;
            
        case PARSE_DATA:
            parse_buffer[parse_index++] = byte;
            if (parse_index >= parse_length + 1) {
                parse_state = PARSE_CRC;
            }
            break;
            
        case PARSE_CRC:
            {
                uint8_t calculated_crc = calculate_crc(parse_buffer, parse_index);
                if (calculated_crc == byte) {
                    frame->cmd = parse_buffer[1];
                    frame->length = parse_length - 1;
                    if (frame->length > 0) {
                        memcpy(frame->data, &parse_buffer[2], frame->length);
                    }
                    frame->valid = true;
                } else {
                    ESP_LOGW(TAG, "CRC error");
                }
                parser_reset();
                return frame->valid;
            }
            break;
    }
    
    return false;
}

/* ============================================
   RESPONSE HANDLERS
   ============================================ */
static void handle_status_response(const uint8_t *data, uint8_t len)
{
    if (len < 14) {
        ESP_LOGW(TAG, "Status response too short: %d", len);
        return;
    }
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    
    cached_status.state = (motor_state_t)data[0];
    cached_status.error = (motor_error_t)data[1];
    cached_status.is_calibrated = (data[2] & 0x01) != 0;
    cached_status.is_homed = (data[2] & 0x02) != 0;
    memcpy(&cached_status.position_steps, &data[3], 4);
    int16_t pct_fixed;
    memcpy(&pct_fixed, &data[7], 2);
    cached_status.position_percent = (float)pct_fixed / 10.0f;
    memcpy(&cached_status.total_steps, &data[9], 4);
    cached_status.sgt_threshold = (int8_t)data[13];
    cached_status.is_connected = true;
    
    xSemaphoreGive(status_mutex);
    
    last_response_time = esp_timer_get_time();
    
    ESP_LOGD(TAG, "Status: pos=%.1f%%, cal=%d, home=%d", 
             cached_status.position_percent,
             cached_status.is_calibrated,
             cached_status.is_homed);
    
    if (status_callback) {
        status_callback(&cached_status);
    }
}

static void handle_position_response(const uint8_t *data, uint8_t len)
{
    if (len < 6) return;
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    
    memcpy(&cached_status.position_steps, data, 4);
    int16_t pct_fixed;
    memcpy(&pct_fixed, &data[4], 2);
    cached_status.position_percent = (float)pct_fixed / 10.0f;
    
    xSemaphoreGive(status_mutex);
    
    last_response_time = esp_timer_get_time();
}

static void handle_sgt_response(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    cached_status.sgt_threshold = (int8_t)data[0];
    xSemaphoreGive(status_mutex);
    
    last_response_time = esp_timer_get_time();
}

static void handle_temperature_response(const uint8_t *data, uint8_t len)
{
    if (len < 8) {
        ESP_LOGW(TAG, "Temperature response too short: %d", len);
        return;
    }
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    memcpy(&cached_object_temp, &data[0], 4);
    memcpy(&cached_ambient_temp, &data[4], 4);
    xSemaphoreGive(status_mutex);
    
    last_response_time = esp_timer_get_time();
    
    ESP_LOGD(TAG, "Temperature: object=%.1f°C, ambient=%.1f°C", 
             cached_object_temp, cached_ambient_temp);
    
    if (temperature_callback) {
        temperature_callback(cached_object_temp, cached_ambient_temp);
    }
}

static void handle_pot_presence_response(const uint8_t *data, uint8_t len)
{
    if (len < 3) {
        ESP_LOGW(TAG, "Pot presence response too short: %d", len);
        return;
    }
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    cached_pot_present = (data[0] != 0);
    memcpy(&cached_pot_distance_mm, &data[1], 2);
    xSemaphoreGive(status_mutex);
    
    last_response_time = esp_timer_get_time();
    
    ESP_LOGD(TAG, "Pot presence: %s, distance=%u mm", 
             cached_pot_present ? "PRESENT" : "NOT PRESENT", cached_pot_distance_mm);
    
    if (pot_presence_callback) {
        pot_presence_callback(cached_pot_present, cached_pot_distance_mm);
    }
}

static void handle_response(proto_frame_t *frame)
{
    last_response_time = esp_timer_get_time();
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    cached_status.is_connected = true;
    xSemaphoreGive(status_mutex);
    
    switch (frame->cmd) {
        case RSP_ACK:
            ESP_LOGD(TAG, "ACK received");
            last_error = 0;
            break;
            
        case RSP_NAK:
            if (frame->length >= 1) {
                last_error = frame->data[0];
                ESP_LOGW(TAG, "NAK received, error: %d", last_error);
                if (error_callback) {
                    error_callback(last_error);
                }
            }
            break;
            
        case RSP_STATUS:
            handle_status_response(frame->data, frame->length);
            break;
            
        case RSP_POSITION:
            handle_position_response(frame->data, frame->length);
            break;
            
        case RSP_SGT:
            handle_sgt_response(frame->data, frame->length);
            break;
            
        case RSP_TEMPERATURE:
            handle_temperature_response(frame->data, frame->length);
            break;
            
        case RSP_POT_PRESENCE:
            handle_pot_presence_response(frame->data, frame->length);
            break;
            
        case RSP_PONG:
            ESP_LOGD(TAG, "PONG received");
            break;
            
        case NOTIFY_MOVE_COMPLETE:
            ESP_LOGI(TAG, "Move complete");
            if (move_complete_callback) {
                move_complete_callback(true);
            }
            break;
            
        case NOTIFY_HOME_COMPLETE:
            ESP_LOGI(TAG, "Home complete");
            xSemaphoreTake(status_mutex, portMAX_DELAY);
            cached_status.is_homed = true;
            xSemaphoreGive(status_mutex);
            if (home_complete_callback) {
                home_complete_callback(true);
            }
            break;
            
        case NOTIFY_CALIBRATE_DONE:
            ESP_LOGI(TAG, "Calibration complete");
            xSemaphoreTake(status_mutex, portMAX_DELAY);
            cached_status.is_calibrated = true;
            xSemaphoreGive(status_mutex);
            if (calibrate_complete_callback) {
                calibrate_complete_callback(true);
            }
            break;
            
        case NOTIFY_ERROR:
            if (frame->length >= 1) {
                ESP_LOGE(TAG, "Motor error: %d", frame->data[0]);
                if (error_callback) {
                    error_callback(frame->data[0]);
                }
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown response: 0x%02X", frame->cmd);
            break;
    }
}

/* ============================================
   UART TASK
   ============================================ */
static void uart_comm_task(void *pvParameters)
{
    uint8_t rx_byte;
    proto_frame_t frame;
    int64_t last_ping_time = 0;
    
    ESP_LOGI(TAG, "UART communication task started");
    parser_reset();
    
    /* Initial status request */
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_comm_get_status();
    
    while (is_running) {
        /* Read incoming data */
        int len = uart_read_bytes(UART_NUM, &rx_byte, 1, pdMS_TO_TICKS(10));
        
        if (len > 0) {
            if (parse_byte(rx_byte, &frame)) {
                if (frame.valid) {
                    handle_response(&frame);
                }
            }
        }
        
        /* Periodic ping to check connection */
        int64_t now = esp_timer_get_time();
        if ((now - last_ping_time) >= (PING_INTERVAL_MS * 1000)) {
            uart_comm_ping();
            last_ping_time = now;
        }
        
        /* Check for connection timeout */
        if ((now - last_response_time) > (PING_TIMEOUT_MS * 1000 * 3)) {
            xSemaphoreTake(status_mutex, portMAX_DELAY);
            if (cached_status.is_connected) {
                cached_status.is_connected = false;
                ESP_LOGW(TAG, "Motor controller disconnected");
            }
            xSemaphoreGive(status_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGI(TAG, "UART communication task stopped");
    uart_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================
   PUBLIC FUNCTIONS - INITIALIZATION
   ============================================ */
esp_err_t uart_comm_init(void)
{
    ESP_LOGI(TAG, "Initializing UART communication...");
    
    /* Create mutex */
    if (status_mutex == NULL) {
        status_mutex = xSemaphoreCreateMutex();
        if (status_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    /* Configure UART */
    uart_config_t uart_config = {
        .baud_rate = UART_COMM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret;
    
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(UART_NUM, UART_COMM_TX_PIN, UART_COMM_RX_PIN, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize cached status */
    memset(&cached_status, 0, sizeof(cached_status));
    cached_status.is_connected = false;
    
    ESP_LOGI(TAG, "UART initialized (TX:%d, RX:%d, %d baud)", 
             UART_COMM_TX_PIN, UART_COMM_RX_PIN, UART_COMM_BAUD);
    
    return ESP_OK;
}

void uart_comm_start(void)
{
    if (is_running) {
        ESP_LOGW(TAG, "UART task already running");
        return;
    }
    
    is_running = true;
    last_response_time = esp_timer_get_time();
    
    BaseType_t ret = xTaskCreate(uart_comm_task, "uart_comm", 4096, NULL, 10, &uart_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART task");
        is_running = false;
    }
}

void uart_comm_task_stop(void)
{
    is_running = false;
    /* Task will delete itself */
}

/* ============================================
   PUBLIC FUNCTIONS - CALLBACKS
   ============================================ */
void uart_comm_set_status_callback(uart_comm_status_cb_t callback)
{
    status_callback = callback;
}

void uart_comm_set_move_complete_callback(uart_comm_move_complete_cb_t callback)
{
    move_complete_callback = callback;
}

void uart_comm_set_home_complete_callback(uart_comm_home_complete_cb_t callback)
{
    home_complete_callback = callback;
}

void uart_comm_set_calibrate_complete_callback(uart_comm_calibrate_complete_cb_t callback)
{
    calibrate_complete_callback = callback;
}

void uart_comm_set_error_callback(uart_comm_error_cb_t callback)
{
    error_callback = callback;
}

void uart_comm_set_temperature_callback(uart_comm_temperature_cb_t callback)
{
    temperature_callback = callback;
}

void uart_comm_set_pot_presence_callback(uart_comm_pot_presence_cb_t callback)
{
    pot_presence_callback = callback;
}

/* ============================================
   PUBLIC FUNCTIONS - COMMANDS
   ============================================ */
bool uart_comm_ping(void)
{
    send_command(CMD_PING, NULL, 0);
    return true;
}

bool uart_comm_get_status(void)
{
    send_command(CMD_GET_STATUS, NULL, 0);
    return true;
}

bool uart_comm_calibrate(void)
{
    ESP_LOGI(TAG, "Sending CALIBRATE command");
    send_command(CMD_CALIBRATE, NULL, 0);
    return true;
}

bool uart_comm_home(void)
{
    ESP_LOGI(TAG, "Sending HOME command");
    send_command(CMD_HOME, NULL, 0);
    return true;
}

bool uart_comm_move_to_percent(float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    int16_t pct_fixed = (int16_t)(percent * 10.0f);
    ESP_LOGI(TAG, "Sending MOVE_PERCENT: %.1f%%", percent);
    send_command(CMD_MOVE_PERCENT, (uint8_t*)&pct_fixed, 2);
    return true;
}

bool uart_comm_move_to_position(int32_t steps)
{
    ESP_LOGI(TAG, "Sending MOVE_POSITION: %ld", (long)steps);
    send_command(CMD_MOVE_POSITION, (uint8_t*)&steps, 4);
    return true;
}

bool uart_comm_motor_stop(void)
{
    ESP_LOGI(TAG, "Sending STOP command");
    send_command(CMD_STOP, NULL, 0);
    return true;
}

bool uart_comm_enable(void)
{
    ESP_LOGI(TAG, "Sending ENABLE command");
    send_command(CMD_ENABLE, NULL, 0);
    return true;
}

bool uart_comm_disable(void)
{
    ESP_LOGI(TAG, "Sending DISABLE command");
    send_command(CMD_DISABLE, NULL, 0);
    return true;
}

bool uart_comm_set_sgt(int8_t sgt)
{
    if (sgt < -64) sgt = -64;
    if (sgt > 63) sgt = 63;
    
    ESP_LOGI(TAG, "Sending SET_SGT: %d", sgt);
    send_command(CMD_SET_SGT, (uint8_t*)&sgt, 1);
    return true;
}

bool uart_comm_get_sgt(void)
{
    send_command(CMD_GET_SGT, NULL, 0);
    return true;
}

bool uart_comm_save_calibration(void)
{
    ESP_LOGI(TAG, "Sending SAVE_CALIBRATION command");
    send_command(CMD_SAVE_CALIBRATION, NULL, 0);
    return true;
}

bool uart_comm_clear_calibration(void)
{
    ESP_LOGI(TAG, "Sending CLEAR_CALIBRATION command");
    send_command(CMD_CLEAR_CALIBRATION, NULL, 0);
    return true;
}

bool uart_comm_get_temperature(void)
{
    send_command(CMD_GET_TEMPERATURE, NULL, 0);
    return true;
}

bool uart_comm_get_pot_presence(void)
{
    send_command(CMD_GET_POT_PRESENCE, NULL, 0);
    return true;
}

/* ============================================
   PUBLIC FUNCTIONS - STATUS
   ============================================ */
void uart_comm_get_cached_status(motor_status_t *status)
{
    if (status == NULL) return;
    
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    memcpy(status, &cached_status, sizeof(motor_status_t));
    xSemaphoreGive(status_mutex);
}

void uart_comm_get_cached_temperature(float *object_temp, float *ambient_temp)
{
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    if (object_temp) *object_temp = cached_object_temp;
    if (ambient_temp) *ambient_temp = cached_ambient_temp;
    xSemaphoreGive(status_mutex);
}

void uart_comm_get_cached_pot_presence(bool *is_present, uint16_t *distance_mm)
{
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    if (is_present) *is_present = cached_pot_present;
    if (distance_mm) *distance_mm = cached_pot_distance_mm;
    xSemaphoreGive(status_mutex);
}

bool uart_comm_is_connected(void)
{
    bool connected;
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    connected = cached_status.is_connected;
    xSemaphoreGive(status_mutex);
    return connected;
}

uint8_t uart_comm_get_last_error(void)
{
    return last_error;
}