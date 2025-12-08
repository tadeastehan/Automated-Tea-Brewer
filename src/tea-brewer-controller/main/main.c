/**
 * @file main.c
 * @brief Tea Brewer Motor Controller - Main Entry Point
 * 
 * This is ESP #2 (Motor Controller) that:
 *   - Controls the stepper motor via TMC2130
 *   - Accepts commands via USB Serial JTAG (human interface)
 *   - Accepts commands via UART (from Display ESP #1)
 *   - Stores calibration in NVS flash
 * 
 * Human Interface (USB Serial JTAG):
 *   c     - Full calibration
 *   f     - Fast home
 *   0-100 - Move to percent
 *   h     - Help
 * 
 * ESP-to-ESP Interface (UART):
 *   Binary protocol (see protocol.h)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "motor/motor_control.h"
#include "console/console.h"
#include "console/uart_comm.h"
#include "temperature_sensor/thermometer.h"
#include "distance_sensor/distance_sensor.h"
#include "pot_sensor/pot_sensor.h"

static const char *TAG = "MAIN";

#define DEBUG 0  // Set to 1 to enable distance sensor debug output

/**
 * @brief Initialize NVS flash storage
 */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS: Erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Print startup banner
 */
static void print_banner(void)
{
    /* Wait for USB to be ready */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    console_printf("\r\n");
    console_printf("╔════════════════════════════════════════╗\r\n");
    console_printf("║   TEA BREWER - MOTOR CONTROLLER        ║\r\n");
    console_printf("║   ESP32-C6 + TMC2130                   ║\r\n");
    console_printf("╠════════════════════════════════════════╣\r\n");
    console_printf("║   USB: Human interface (text)          ║\r\n");
    console_printf("║   UART: Display ESP (binary protocol)  ║\r\n");
    console_printf("╚════════════════════════════════════════╝\r\n");
    console_printf("\r\n");
}

/**
 * @brief Print current system status
 */
static void print_startup_status(void)
{
    motor_status_t status;
    motor_get_status(&status);
    
    console_printf("=== System Status ===\r\n");
    console_printf("Motor driver:   %s\r\n", motor_test_connection() ? "OK" : "FAILED");
    console_printf("Calibration:    %s\r\n", status.is_calibrated ? "LOADED" : "NONE");
    
    if (status.is_calibrated) {
        console_printf("  Total steps:  %ld\r\n", (long)status.total_steps);
        console_printf("  SGT:          %d\r\n", status.sgt_threshold);
    }
    
    console_printf("=====================\r\n");
    console_printf("\r\n");
    
    if (status.is_calibrated) {
        console_printf("Calibration found. Type 'f' for fast home.\r\n");
    } else {
        console_printf("No calibration. Type 'c' to calibrate (without load).\r\n");
    }
    
    console_printf("Type 'h' for help.\r\n");
    console_printf("\r\n");
}

/**
 * @brief Distance sensor reading task
 * 
 * Reads and prints distance every 250ms and shows pot status
 */
static void distance_task(void *arg)
{
    uint16_t distance_mm;
    bool pot_present;
    
    while (1) {
        if (pot_sensor_is_initialized()) {
            esp_err_t ret = pot_sensor_get_distance(&distance_mm);
            if (ret == ESP_OK) {
                pot_present = pot_sensor_is_present();
                console_printf("Distance: %u mm | Pot: %s\r\n", 
                              distance_mm, pot_present ? "PRESENT" : "NOT PRESENT");
            } else {
                console_printf("Distance: ERROR\r\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/**
 * @brief Application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Tea Brewer Motor Controller starting...");
    
    /* ========================================
       STEP 1: Initialize NVS Flash
       ======================================== */
    ESP_LOGI(TAG, "Initializing NVS...");
    ESP_ERROR_CHECK(init_nvs());
    
    /* ========================================
       STEP 2: Initialize Console (USB Serial JTAG)
       ======================================== */
    ESP_LOGI(TAG, "Initializing console...");
    ESP_ERROR_CHECK(console_init());
    
    /* Print startup banner */
    print_banner();
    
    /* ========================================
       STEP 3: Initialize Motor Control
       ======================================== */
    console_printf("Initializing motor controller...\r\n");
    esp_err_t ret = motor_init();
    if (ret != ESP_OK) {
        console_printf("ERROR: Motor init failed!\r\n");
        ESP_LOGE(TAG, "Motor initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    console_printf("Motor controller: OK\r\n");
    
    /* ========================================
       STEP 4: Load Calibration from Flash
       ======================================== */
    console_printf("Loading calibration...\r\n");
    ret = motor_load_calibration();
    bool calibration_loaded = (ret == ESP_OK);
    if (calibration_loaded) {
        console_printf("Calibration: LOADED\r\n");
    } else {
        console_printf("Calibration: NOT FOUND\r\n");
    }
    
    /* NOTE: No automatic homing on startup
     * ESP #2 waits in IDLE state for ESP #1 to send HOME command */
    
    /* ========================================
       STEP 5: Initialize Temperature Sensor
       ======================================== */
    console_printf("Initializing temperature sensor...\r\n");
    ret = thermometer_init();
    if (ret == ESP_OK) {
        console_printf("Temperature sensor: OK\r\n");
    } else {
        console_printf("Temperature sensor: FAILED\r\n");
        ESP_LOGW(TAG, "Temperature sensor init failed: %s", esp_err_to_name(ret));
    }
    
    /* ========================================
       STEP 5.5: Initialize Distance Sensor (VL53L0X)
       ======================================== */
    console_printf("Initializing distance sensor...\r\n");
    /* Share I2C bus with temperature sensor if available */
    i2c_master_bus_handle_t i2c_bus = thermometer_get_bus_handle();
    ret = distance_sensor_init(i2c_bus);
    if (ret == ESP_OK) {
        console_printf("Distance sensor: OK\r\n");
    } else {
        console_printf("Distance sensor: FAILED\r\n");
        ESP_LOGW(TAG, "Distance sensor init failed: %s", esp_err_to_name(ret));
    }
    
    /* ========================================
       STEP 5.6: Initialize Pot Sensor
       ======================================== */
    console_printf("Initializing pot sensor...\r\n");
    ret = pot_sensor_init();
    if (ret == ESP_OK) {
        console_printf("Pot sensor: OK (threshold: %u mm)\r\n", pot_sensor_get_threshold());
    } else {
        console_printf("Pot sensor: FAILED\r\n");
        ESP_LOGW(TAG, "Pot sensor init failed: %s", esp_err_to_name(ret));
    }
    
    /* ========================================
       STEP 6: Initialize UART Communication
       ======================================== */
    console_printf("Initializing UART...\r\n");
    ret = uart_comm_init();
    if (ret != ESP_OK) {
        console_printf("ERROR: UART init failed!\r\n");
        ESP_LOGE(TAG, "UART initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    console_printf("UART: OK (TX:16, RX:17, 115200 baud)\r\n");
    
    console_printf("\r\n");
    
    /* ========================================
       STEP 7: Print Status
       ======================================== */
    print_startup_status();
    
    /* ========================================
       STEP 8: Start Communication Tasks
       ======================================== */
    ESP_LOGI(TAG, "Starting tasks...");
    
    /* Start console task (USB Serial JTAG - human interface) */
    console_start_task();
    
    /* Start UART task (ESP-to-ESP binary protocol) */
    uart_comm_start_task();
    
    /* Start distance sensor reading task (only if DEBUG enabled) */
#if DEBUG
    if (pot_sensor_is_initialized()) {
        xTaskCreate(distance_task, "distance_task", 2048, NULL, 5, NULL);
        ESP_LOGI(TAG, "Pot sensor task started (250ms interval)");
    }
#endif
    
    ESP_LOGI(TAG, "System ready!");
    
    /* ========================================
       MAIN LOOP - Idle
       ======================================== */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}