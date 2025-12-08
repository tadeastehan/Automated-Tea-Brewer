/**
 * @file rtc.h
 * @brief DS1307 Real-Time Clock interface
 */

#ifndef RTC_H
#define RTC_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <time.h>
#include <stdbool.h>

/**
 * @brief Initialize DS1307 RTC with shared I2C bus
 * @param i2c_bus I2C master bus handle (shared with other sensors)
 * @return ESP_OK on success
 */
esp_err_t rtc_init(i2c_master_bus_handle_t i2c_bus);

/**
 * @brief Get current time from RTC
 * @param time Pointer to tm structure to store the time
 * @return ESP_OK on success
 */
esp_err_t rtc_get_time(struct tm *time);

/**
 * @brief Set RTC time
 * @param time Pointer to tm structure containing the time to set
 * @return ESP_OK on success
 */
esp_err_t rtc_set_time(const struct tm *time);

/**
 * @brief Get current time from RTC adjusted for timezone (GMT+1)
 * @param time Pointer to tm structure to store the adjusted time
 * @return ESP_OK on success
 */
esp_err_t rtc_get_time_with_timezone(struct tm *time);

/**
 * @brief Check if RTC is initialized
 * @return true if initialized, false otherwise
 */
bool rtc_is_initialized(void);

#endif // RTC_H
