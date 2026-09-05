/**
 * @file d8563.h
 * @brief D8563TS / PCF8563 Real-Time Clock I2C Driver
 */

#ifndef D8563_H
#define D8563_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define D8563_I2C_ADDR          0x51    // Standard 7-bit I2C address for D8563TS / PCF8563

// D8563 Register Map
#define D8563_REG_CTRL1         0x00
#define D8563_REG_CTRL2         0x01
#define D8563_REG_VL_SECONDS    0x02
#define D8563_REG_MINUTES       0x03
#define D8563_REG_HOURS         0x04
#define D8563_REG_DAYS          0x05
#define D8563_REG_WEEKDAYS      0x06
#define D8563_REG_CENTURY_MONTHS 0x07
#define D8563_REG_YEARS         0x08
#define D8563_REG_ALARM_MIN     0x09
#define D8563_REG_ALARM_HOUR    0x0A
#define D8563_REG_ALARM_DAY     0x0B
#define D8563_REG_ALARM_WDAY    0x0C
#define D8563_REG_CLKOUT_CTRL   0x0D
#define D8563_REG_TIMER_CTRL    0x0E
#define D8563_REG_TIMER_VAL     0x0F

typedef struct {
    int year;           // Year (e.g. 2026)
    int month;          // Month 1-12
    int day;            // Day 1-31
    int weekday;        // Weekday 0-6 (0=Sunday, 1=Monday, ...)
    int hour;           // Hour 0-23
    int minute;         // Minute 0-59
    int second;         // Second 0-59
    bool voltage_low;   // true if battery voltage is low / power loss occurred (VL bit = 1)
} d8563_time_t;

typedef struct {
    uint8_t raw_regs[16];
    bool is_present;
    bool is_running;
    bool voltage_low;
    d8563_time_t time;
} d8563_status_t;

/**
 * @brief Initialize D8563TS RTC on the specified I2C master bus
 * @param sda_pin GPIO pin for SDA
 * @param scl_pin GPIO pin for SCL
 * @param speed_hz I2C clock speed in Hz (e.g. 50000 or 100000)
 * @return ESP_OK on success
 */
esp_err_t d8563_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t speed_hz);

/**
 * @brief Re-initialize I2C bus with new SDA and SCL pins and speed
 */
esp_err_t d8563_reinit_pins(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t speed_hz);

/**
 * @brief Send 9 clock pulses on SCL to unstick any slave holding SDA low
 */
void d8563_recover_bus(gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief Check if D8563 is present on the I2C bus
 * @return true if responding at address 0x51
 */
bool d8563_is_connected(void);

/**
 * @brief Read current time and date from D8563
 * @param time Pointer to d8563_time_t struct to receive time
 * @return ESP_OK on success
 */
esp_err_t d8563_get_time(d8563_time_t *time);

/**
 * @brief Set time and date in D8563 (clears STOP and VL flags)
 * @param time Pointer to time structure with desired values
 * @return ESP_OK on success
 */
esp_err_t d8563_set_time(const d8563_time_t *time);

/**
 * @brief Read all 16 registers and status from D8563
 * @param status Pointer to d8563_status_t to receive full report
 * @return ESP_OK on success
 */
esp_err_t d8563_get_status(d8563_status_t *status);

/**
 * @brief Scan the I2C bus using hardware peripheral
 */
void d8563_scan_i2c_bus(void);

/**
 * @brief Scan the I2C bus using slow, robust software bit-banging
 */
void d8563_bitbang_scan(void);

/**
 * @brief Direct bit-bang ping of a single 7-bit I2C address with step-by-step trace
 */
void d8563_bitbang_ping(uint8_t addr_7bit);

/**
 * @brief Diagnostic check on physical pin electrical states (pull-ups)
 */
void d8563_diagnose_bus(void);

/**
 * @brief Get current SDA, SCL, and speed
 */
void d8563_get_pins(gpio_num_t *sda, gpio_num_t *scl, uint32_t *speed_hz);

/**
 * @brief Get weekday name string from 0..6 index
 */
const char *d8563_weekday_name(int wday);

#ifdef __cplusplus
}
#endif

#endif // D8563_H
