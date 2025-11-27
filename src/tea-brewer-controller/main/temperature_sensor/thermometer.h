/**
 * @file thermometer.h
 * @brief MLX90614 Infrared Temperature Sensor Driver
 * 
 * This module provides an interface to the MLX90614 infrared thermometer
 * using the I2C master driver. It supports reading both object (target)
 * and ambient temperatures.
 * 
 * Usage:
 *   1. Call thermometer_init() once at startup
 *   2. Use thermometer_get_object_temp() to read target temperature
 *   3. Use thermometer_get_ambient_temp() to read ambient temperature
 *   4. Use thermometer_get_temperatures() to read both at once
 */

#ifndef THERMOMETER_H
#define THERMOMETER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Temperature readings structure
 */
typedef struct {
    float object_temp;   /**< Object (target) temperature in °C */
    float ambient_temp;  /**< Ambient temperature in °C */
} thermometer_readings_t;

/**
 * @brief Initialize the thermometer (MLX90614) sensor
 * 
 * This function initializes the I2C bus and configures the MLX90614 sensor.
 * Must be called before any other thermometer functions.
 * 
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if already initialized
 *      - ESP_FAIL on I2C or sensor initialization failure
 */
esp_err_t thermometer_init(void);

/**
 * @brief Deinitialize the thermometer sensor
 * 
 * Releases I2C resources. Call this when thermometer is no longer needed.
 * 
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t thermometer_deinit(void);

/**
 * @brief Get the object (target) temperature
 * 
 * Reads the temperature of the object the sensor is pointed at.
 * 
 * @param[out] temperature Pointer to store temperature in °C
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if temperature is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_FAIL on read failure
 */
esp_err_t thermometer_get_object_temp(float *temperature);

/**
 * @brief Get the ambient temperature
 * 
 * Reads the ambient temperature around the sensor.
 * 
 * @param[out] temperature Pointer to store temperature in °C
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if temperature is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_FAIL on read failure
 */
esp_err_t thermometer_get_ambient_temp(float *temperature);

/**
 * @brief Get both object and ambient temperatures
 * 
 * Convenience function to read both temperatures in one call.
 * 
 * @param[out] readings Pointer to store both temperature readings
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if readings is NULL
 *      - ESP_ERR_INVALID_STATE if not initialized
 *      - ESP_FAIL on read failure
 */
esp_err_t thermometer_get_temperatures(thermometer_readings_t *readings);

/**
 * @brief Check if the thermometer is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool thermometer_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif // THERMOMETER_H
