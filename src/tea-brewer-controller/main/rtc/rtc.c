/**
 * @file rtc.c
 * @brief DS1307 Real-Time Clock implementation using ESP-IDF I2C master driver
 */

#include "rtc.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RTC";

#define DS1307_ADDR         0x68    // DS1307 I2C address
#define DS1307_REG_TIME     0x00    // Time register start address

static i2c_master_dev_handle_t rtc_handle = NULL;
static bool initialized = false;

/* Helper: BCD to Decimal conversion */
static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

/* Helper: Decimal to BCD conversion */
static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

esp_err_t rtc_init(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "Initializing DS1307 RTC...");
    
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Configure DS1307 device on shared I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS1307_ADDR,
        .scl_speed_hz = 100000,  // 100kHz
    };
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &rtc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DS1307 to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "DS1307 RTC initialized successfully (shared I2C bus)");
    
    /* Uncomment to set initial time:
    struct tm time = {
        .tm_year = 124,  // 2024 - 1900
        .tm_mon  = 11,   // December (0-based)
        .tm_mday = 8,
        .tm_hour = 12,
        .tm_min  = 0,
        .tm_sec  = 0
    };
    ret = rtc_set_time(&time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set time: %s", esp_err_to_name(ret));
    }
    */
    
    return ESP_OK;
}

esp_err_t rtc_get_time(struct tm *time)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[7];
    uint8_t reg_addr = DS1307_REG_TIME;
    
    /* Write register address then read 7 bytes */
    esp_err_t ret = i2c_master_transmit_receive(rtc_handle, &reg_addr, 1, data, 7, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    /* Convert BCD to decimal */
    time->tm_sec = bcd2dec(data[0] & 0x7F);  // Mask CH bit
    time->tm_min = bcd2dec(data[1]);
    time->tm_hour = bcd2dec(data[2] & 0x3F); // 24-hour format
    time->tm_wday = bcd2dec(data[3]) - 1;    // 0-6
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon = bcd2dec(data[5]) - 1;     // 0-11
    time->tm_year = bcd2dec(data[6]) + 100;  // Years since 1900 (2000-2099 -> 100-199)
    
    return ESP_OK;
}

esp_err_t rtc_set_time(const struct tm *time)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[8];
    
    data[0] = DS1307_REG_TIME;                      // Register address
    data[1] = dec2bcd(time->tm_sec) & 0x7F;        // Seconds (ensure CH=0, clock enabled)
    data[2] = dec2bcd(time->tm_min);               // Minutes
    data[3] = dec2bcd(time->tm_hour);              // Hours (24-hour format)
    data[4] = dec2bcd(time->tm_wday + 1);          // Day of week (1-7)
    data[5] = dec2bcd(time->tm_mday);              // Day of month
    data[6] = dec2bcd(time->tm_mon + 1);           // Month (1-12)
    data[7] = dec2bcd(time->tm_year % 100);        // Year (00-99)
    
    return i2c_master_transmit(rtc_handle, data, 8, -1);
}

esp_err_t rtc_get_time_with_timezone(struct tm *time)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Get UTC time from RTC */
    esp_err_t ret = rtc_get_time(time);
    if (ret != ESP_OK) {
        return ret;
    }
    
    /* Convert to time_t for easy calculation */
    time_t raw_time = mktime(time);
    
    /* Add GMT+1 offset (3600 seconds = 1 hour) */
    raw_time += 3600;
    
    /* Convert back to tm structure */
    struct tm *adjusted_time = localtime(&raw_time);
    if (adjusted_time == NULL) {
        return ESP_FAIL;
    }
    
    /* Copy adjusted time back */
    memcpy(time, adjusted_time, sizeof(struct tm));
    
    return ESP_OK;
}

bool rtc_is_initialized(void)
{
    return initialized;
}
