/**
 * @file distance_sensor.h
 * @brief VL53L0X Time-of-Flight Distance Sensor Driver
 * 
 * This module provides an interface to the VL53L0X ToF sensor
 * using the new ESP-IDF I2C master driver.
 */

#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the distance sensor (VL53L0X)
 * 
 * @param bus_handle Existing I2C bus handle (NULL to create new bus)
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if already initialized
 *      - ESP_FAIL on initialization failure
 */
esp_err_t distance_sensor_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Deinitialize the distance sensor
 * 
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t distance_sensor_deinit(void);

/**
 * @brief Get distance reading in millimeters
 * 
 * @param[out] distance_mm Pointer to store distance in mm
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if distance_mm is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_ERR_TIMEOUT if measurement timed out
 *      - ESP_FAIL on read failure
 */
esp_err_t distance_sensor_get_distance(uint16_t *distance_mm);

/**
 * @brief Start continuous measurement mode
 * 
 * @param period_ms Measurement period (0 for back-to-back mode)
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t distance_sensor_start_continuous(uint32_t period_ms);

/**
 * @brief Stop continuous measurement mode
 * 
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t distance_sensor_stop_continuous(void);

/**
 * @brief Read distance in continuous mode
 * 
 * @param[out] distance_mm Pointer to store distance in mm
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if distance_mm is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_ERR_TIMEOUT if measurement timed out
 */
esp_err_t distance_sensor_read_continuous(uint16_t *distance_mm);

/**
 * @brief Check if the distance sensor is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool distance_sensor_is_initialized(void);

/**
 * @brief Get the I2C bus handle used by the sensor
 * 
 * @return I2C bus handle or NULL if not initialized
 */
i2c_master_bus_handle_t distance_sensor_get_bus_handle(void);

#ifdef __cplusplus
}
#endif

#endif // DISTANCE_SENSOR_H
