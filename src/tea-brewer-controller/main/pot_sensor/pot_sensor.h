/**
 * @file pot_sensor.h
 * @brief Tea Pot Detection Module
 * 
 * This module provides functionality to detect if the tea pot
 * is in position using the VL53L0X distance sensor.
 */

#ifndef POT_SENSOR_H
#define POT_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default threshold distance in mm - pot is present if distance < this value */
#define POT_THRESHOLD_DEFAULT_MM    110

/**
 * @brief Initialize the pot sensor module
 * 
 * Must be called after distance_sensor_init()
 * 
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if distance sensor not initialized
 */
esp_err_t pot_sensor_init(void);

/**
 * @brief Check if the tea pot is in position
 * 
 * Reads the distance sensor and compares against the threshold.
 * Pot is considered present if distance < threshold.
 * 
 * @param[out] is_present Pointer to store result (true if pot is present)
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if is_present is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_FAIL on sensor read failure
 */
esp_err_t pot_sensor_check(bool *is_present);

/**
 * @brief Check if pot is present (simple version)
 * 
 * @return true if pot is in position, false otherwise or on error
 */
bool pot_sensor_is_present(void);

/**
 * @brief Get the current distance to the pot (or empty space)
 * 
 * @param[out] distance_mm Pointer to store distance in mm
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if distance_mm is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_FAIL on sensor read failure
 */
esp_err_t pot_sensor_get_distance(uint16_t *distance_mm);

/**
 * @brief Set the detection threshold
 * 
 * @param threshold_mm Distance threshold in mm (pot present if distance < threshold)
 */
void pot_sensor_set_threshold(uint16_t threshold_mm);

/**
 * @brief Get the current detection threshold
 * 
 * @return Current threshold in mm
 */
uint16_t pot_sensor_get_threshold(void);

/**
 * @brief Check if pot sensor module is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool pot_sensor_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif // POT_SENSOR_H
