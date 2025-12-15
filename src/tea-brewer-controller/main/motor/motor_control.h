/**
 * @file motor_control.h
 * @brief High-level motor control API
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ============================================
   MOTOR STATUS
   ============================================ */
typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_HOMING,
    MOTOR_STATE_CALIBRATING,
    MOTOR_STATE_ERROR
} motor_state_t;

typedef enum
{
    MOTOR_ERROR_NONE = 0,
    MOTOR_ERROR_NOT_CALIBRATED,
    MOTOR_ERROR_NOT_HOMED,
    MOTOR_ERROR_STALL_DETECTED,
    MOTOR_ERROR_DRIVER_FAULT,
    MOTOR_ERROR_OVERTEMP
} motor_error_t;

typedef struct
{
    motor_state_t state;
    motor_error_t error;
    bool is_calibrated;
    bool is_homed;
    int32_t position_steps;
    float position_percent;
    int32_t total_steps;
    int8_t sgt_threshold;
} motor_status_t;

/* ============================================
   INITIALIZATION
   ============================================ */

/**
 * @brief Initialize motor control system
 * @return ESP_OK on success
 */
esp_err_t motor_init(void);

/**
 * @brief Load calibration from NVS flash
 * @return ESP_OK if calibration found and loaded
 */
esp_err_t motor_load_calibration(void);

/**
 * @brief Save calibration to NVS flash
 * @return ESP_OK on success
 */
esp_err_t motor_save_calibration(void);

/**
 * @brief Clear calibration from NVS flash
 * @return ESP_OK on success
 */
esp_err_t motor_clear_calibration(void);

/* ============================================
   CALIBRATION & HOMING
   ============================================ */

/**
 * @brief Perform full calibration (finds both endpoints)
 * @note Run WITHOUT load!
 * @return ESP_OK on success
 */
esp_err_t motor_calibrate(void);

/**
 * @brief Perform calibration with StallGuard monitoring
 * @note Run WITHOUT load! Displays SG values during calibration
 * @return ESP_OK on success
 */
esp_err_t motor_calibrate_with_sg_monitor(void);

/**
 * @brief Perform fast home (finds start position only)
 * @note Can be run with load
 * @return ESP_OK on success
 */
esp_err_t motor_home(void);

/* ============================================
   MOVEMENT
   ============================================ */

/**
 * @brief Move to percentage position
 * @param percent Position 0-100%
 * @return ESP_OK on success
 */
esp_err_t motor_move_to_percent(float percent);

/**
 * @brief Move to absolute step position
 * @param steps Target position in steps
 * @return ESP_OK on success
 */
esp_err_t motor_move_to_position(int32_t steps);

/**
 * @brief Stop motor immediately
 */
void motor_stop(void);

/**
 * @brief Enable motor driver
 */
void motor_enable(void);

/**
 * @brief Disable motor driver
 */
void motor_disable(void);

/* ============================================
   CONFIGURATION
   ============================================ */

/**
 * @brief Set StallGuard threshold
 * @param threshold Value from -64 to 63
 */
void motor_set_sgt(int8_t threshold);

/**
 * @brief Get current StallGuard threshold
 * @return Current threshold
 */
int8_t motor_get_sgt(void);

/**
 * @brief Increase StallGuard threshold (less sensitive)
 * @return New threshold value
 */
int8_t motor_increase_sgt(void);

/**
 * @brief Decrease StallGuard threshold (more sensitive)
 * @return New threshold value
 */
int8_t motor_decrease_sgt(void);

/* ============================================
   STATUS
   ============================================ */

/**
 * @brief Get current motor status
 * @param status Pointer to status structure to fill
 */
void motor_get_status(motor_status_t *status);

/**
 * @brief Check if motor is currently moving
 * @return true if moving
 */
bool motor_is_moving(void);

/**
 * @brief Get current position as percentage
 * @return Position 0-100%, or -1 if not homed
 */
float motor_get_position_percent(void);

/**
 * @brief Get current position in steps
 * @return Position in steps
 */
int32_t motor_get_position_steps(void);

/* ============================================
   DIAGNOSTICS
   ============================================ */

/**
 * @brief Test StallGuard - read current SG value
 * @return Current StallGuard result (0-1023)
 */
uint16_t motor_read_stallguard(void);

/**
 * @brief Check driver connection
 * @return true if driver responds correctly
 */
bool motor_test_connection(void);

#endif // MOTOR_CONTROL_H