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

/**
 * @brief Set motor speed in RPM
 * @param rpm Speed in RPM (10 - 800)
 * @return ESP_OK on success
 */
esp_err_t motor_set_speed_rpm(uint32_t rpm);

/**
 * @brief Get current motor speed in RPM
 * @return Current speed in RPM
 */
uint32_t motor_get_speed_rpm(void);

/**
 * @brief Execute complete teabag dropoff sequence
 * 
 * Moves to high_percent, shakes between low_percent and high_percent
 * for cycles times at the given speed and delay.
 * 
 * @param cycles Number of shake cycles
 * @param low_percent Lower shake position %
 * @param high_percent Upper shake position % (supports > 100% like 101.0%)
 * @param delay_ms Pause between shakes in ms (0 = immediate/aggressive)
 * @param speed_rpm Speed during sequence in RPM (0 = default normal speed)
 * @return ESP_OK on success
 */
esp_err_t motor_execute_dropoff(uint8_t cycles, float low_percent, float high_percent, uint16_t delay_ms, uint16_t speed_rpm);

/* ============================================
   TEABAG DROPOFF CONFIGURATION
   ============================================ */

typedef struct {
    uint8_t cycles;         // Number of shake cycles (default: 3)
    float low_percent;      // Lower position % (default: 98.0f)
    float high_percent;     // Upper position % (default: 101.0f)
    uint16_t delay_ms;      // Delay between shakes in ms (default: 0)
    uint16_t speed_rpm;     // Motor speed in RPM (default: 180)
} motor_dropoff_config_t;

/**
 * @brief Get current dropoff configuration
 * @param config Pointer to store configuration
 */
void motor_get_dropoff_config(motor_dropoff_config_t *config);

/**
 * @brief Set dropoff configuration in RAM
 * @param config New configuration
 * @return ESP_OK on success
 */
esp_err_t motor_set_dropoff_config(const motor_dropoff_config_t *config);

/**
 * @brief Save dropoff configuration to NVS
 * @return ESP_OK on success
 */
esp_err_t motor_save_dropoff_config(void);

/**
 * @brief Load dropoff configuration from NVS
 * @return ESP_OK on success
 */
esp_err_t motor_load_dropoff_config(void);

/**
 * @brief Reset dropoff configuration to defaults
 */
void motor_reset_dropoff_config(void);

/**
 * @brief Execute dropoff sequence using current stored configuration
 * @return ESP_OK on success
 */
esp_err_t motor_execute_configured_dropoff(void);

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