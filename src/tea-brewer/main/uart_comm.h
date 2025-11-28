/**
 * @file uart_comm.h
 * @brief UART communication with Motor Controller (ESP #2)
 * 
 * This module handles bidirectional communication between:
 *   - ESP #1 (Display/LVGL) - this device
 *   - ESP #2 (Motor Controller)
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================
   CONFIGURATION
   ============================================ */
#define UART_COMM_TX_PIN        43      /* Adjust to your wiring */
#define UART_COMM_RX_PIN        44      /* Adjust to your wiring */
#define UART_COMM_BAUD          115200

/* ============================================
   MOTOR STATUS STRUCTURE
   ============================================ */
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_HOMING,
    MOTOR_STATE_CALIBRATING,
    MOTOR_STATE_ERROR
} motor_state_t;

typedef enum {
    MOTOR_ERROR_NONE = 0,
    MOTOR_ERROR_NOT_CALIBRATED,
    MOTOR_ERROR_NOT_HOMED,
    MOTOR_ERROR_STALL_DETECTED,
    MOTOR_ERROR_DRIVER_FAULT,
    MOTOR_ERROR_OVERTEMP
} motor_error_t;

typedef struct {
    motor_state_t state;
    motor_error_t error;
    bool is_calibrated;
    bool is_homed;
    int32_t position_steps;
    float position_percent;
    int32_t total_steps;
    int8_t sgt_threshold;
    bool is_connected;
} motor_status_t;

/* ============================================
   CALLBACK TYPES
   ============================================ */

/**
 * @brief Callback for status updates
 */
typedef void (*uart_comm_status_cb_t)(const motor_status_t *status);

/**
 * @brief Callback for move complete notification
 */
typedef void (*uart_comm_move_complete_cb_t)(bool success);

/**
 * @brief Callback for home complete notification
 */
typedef void (*uart_comm_home_complete_cb_t)(bool success);

/**
 * @brief Callback for calibration complete notification
 */
typedef void (*uart_comm_calibrate_complete_cb_t)(bool success);

/**
 * @brief Callback for error notification
 */
typedef void (*uart_comm_error_cb_t)(uint8_t error_code);

/**
 * @brief Callback for temperature updates
 * @param object_temp Object temperature in °C
 * @param ambient_temp Ambient temperature in °C
 */
typedef void (*uart_comm_temperature_cb_t)(float object_temp, float ambient_temp);

/**
 * @brief Callback for pot presence updates
 * @param is_present true if pot is detected
 * @param distance_mm Current distance reading in mm
 */
typedef void (*uart_comm_pot_presence_cb_t)(bool is_present, uint16_t distance_mm);

/* ============================================
   INITIALIZATION
   ============================================ */

/**
 * @brief Initialize UART communication
 * @return ESP_OK on success
 */
esp_err_t uart_comm_init(void);

/**
 * @brief Start UART communication task
 */
void uart_comm_start(void);

/**
 * @brief Stop UART communication task
 */
void uart_comm_task_stop(void);

/* ============================================
   CALLBACK REGISTRATION
   ============================================ */

/**
 * @brief Register callback for status updates
 */
void uart_comm_set_status_callback(uart_comm_status_cb_t callback);

/**
 * @brief Register callback for move complete
 */
void uart_comm_set_move_complete_callback(uart_comm_move_complete_cb_t callback);

/**
 * @brief Register callback for home complete
 */
void uart_comm_set_home_complete_callback(uart_comm_home_complete_cb_t callback);

/**
 * @brief Register callback for calibration complete
 */
void uart_comm_set_calibrate_complete_callback(uart_comm_calibrate_complete_cb_t callback);

/**
 * @brief Register callback for errors
 */
void uart_comm_set_error_callback(uart_comm_error_cb_t callback);

/**
 * @brief Register callback for temperature updates
 */
void uart_comm_set_temperature_callback(uart_comm_temperature_cb_t callback);

/**
 * @brief Register callback for pot presence updates
 */
void uart_comm_set_pot_presence_callback(uart_comm_pot_presence_cb_t callback);

/* ============================================
   MOTOR COMMANDS (Non-blocking)
   ============================================ */

/**
 * @brief Ping motor controller
 * @return true if command sent
 */
bool uart_comm_ping(void);

/**
 * @brief Request motor status
 * @return true if command sent
 */
bool uart_comm_get_status(void);

/**
 * @brief Start calibration (run without load!)
 * @return true if command sent
 */
bool uart_comm_calibrate(void);

/**
 * @brief Start fast home
 * @return true if command sent
 */
bool uart_comm_home(void);

/**
 * @brief Move to percentage position
 * @param percent Target position 0-100%
 * @return true if command sent
 */
bool uart_comm_move_to_percent(float percent);

/**
 * @brief Move to absolute step position
 * @param steps Target position in steps
 * @return true if command sent
 */
bool uart_comm_move_to_position(int32_t steps);

/**
 * @brief Stop motor immediately
 * @return true if command sent
 */
bool uart_comm_motor_stop(void);

/**
 * @brief Enable motor driver
 * @return true if command sent
 */
bool uart_comm_enable(void);

/**
 * @brief Disable motor driver
 * @return true if command sent
 */
bool uart_comm_disable(void);

/**
 * @brief Set StallGuard threshold
 * @param sgt Threshold value (-64 to 63)
 * @return true if command sent
 */
bool uart_comm_set_sgt(int8_t sgt);

/**
 * @brief Get StallGuard threshold
 * @return true if command sent
 */
bool uart_comm_get_sgt(void);

/**
 * @brief Save calibration to flash
 * @return true if command sent
 */
bool uart_comm_save_calibration(void);

/**
 * @brief Clear calibration from flash
 * @return true if command sent
 */
bool uart_comm_clear_calibration(void);

/**
 * @brief Request temperature reading
 * @return true if command sent
 */
bool uart_comm_get_temperature(void);

/**
 * @brief Request pot presence status
 * @return true if command sent
 */
bool uart_comm_get_pot_presence(void);

/**
 * @brief Turn on induction cooker
 * @return true if command sent
 */
bool uart_comm_induction_on(void);

/**
 * @brief Turn off induction cooker
 * @return true if command sent
 */
bool uart_comm_induction_off(void);

/* ============================================
   STATUS ACCESS
   ============================================ */

/**
 * @brief Get cached motor status
 * @param status Pointer to status structure to fill
 */
void uart_comm_get_cached_status(motor_status_t *status);

/**
 * @brief Get cached temperature readings
 * @param object_temp Pointer to store object temperature (can be NULL)
 * @param ambient_temp Pointer to store ambient temperature (can be NULL)
 */
void uart_comm_get_cached_temperature(float *object_temp, float *ambient_temp);

/**
 * @brief Get cached pot presence status
 * @param is_present Pointer to store presence status (can be NULL)
 * @param distance_mm Pointer to store distance in mm (can be NULL)
 */
void uart_comm_get_cached_pot_presence(bool *is_present, uint16_t *distance_mm);

/**
 * @brief Check if motor controller is connected
 * @return true if connected and responding
 */
bool uart_comm_is_connected(void);

/**
 * @brief Get last communication error
 * @return Error code or 0 if no error
 */
uint8_t uart_comm_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif // UART_COMM_H