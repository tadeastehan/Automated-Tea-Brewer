/**
 * @file mlx90614.h
 * @brief MLX90614 Non-Contact Infrared Temperature Sensor Driver
 * 
 * Target part: MLX90614ESF-DCC-000-TU (3V Medical/Industrial accuracy)
 * Standard SMBus/I2C address: 0x5A
 */

#ifndef MLX90614_H
#define MLX90614_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MLX90614_I2C_ADDR           0x5A

// MLX90614 RAM Registers
#define MLX90614_RAW_DATA1          0x04
#define MLX90614_RAW_DATA2          0x05
#define MLX90614_TA                 0x06    // Ambient temperature
#define MLX90614_TOBJ1              0x07    // Object 1 temperature
#define MLX90614_TOBJ2              0x08    // Object 2 temperature

// MLX90614 EEPROM Registers
#define MLX90614_TOMAX              0x20
#define MLX90614_TOMIN              0x21
#define MLX90614_PWMCTRL            0x22
#define MLX90614_TARANGE            0x23
#define MLX90614_EMISSIVITY         0x24
#define MLX90614_CONFIG1            0x25
#define MLX90614_ID1                0x3C
#define MLX90614_ID2                0x3D
#define MLX90614_ID3                0x3E
#define MLX90614_ID4                0x3F

typedef struct {
    float ambient_c;            // Ambient temperature in Celsius
    float object_raw_c;         // Raw object temperature in Celsius
    float object_calib_c;       // Calibrated object temperature in Celsius
    uint16_t raw_ambient_u16;
    uint16_t raw_object_u16;
    bool is_present;
} mlx90614_readings_t;

/**
 * @brief Initialize MLX90614 sensor on the specified SDA/SCL pins
 */
esp_err_t mlx90614_init(gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief Check if MLX90614 is connected and responding at address 0x5A
 */
bool mlx90614_is_connected(void);

/**
 * @brief Read ambient and raw object temperatures
 * @param[out] ambient_temp Ambient temp in °C (NULL to ignore)
 * @param[out] object_temp Raw object temp in °C (NULL to ignore)
 * @return ESP_OK on success
 */
esp_err_t mlx90614_read_temp(float *ambient_temp, float *object_temp);

/**
 * @brief Read all temperatures including calibration
 * @param[out] readings Pointer to mlx90614_readings_t structure
 * @return ESP_OK on success
 */
esp_err_t mlx90614_get_readings(mlx90614_readings_t *readings);

/**
 * @brief Apply empirical polynomial calibration curve to raw IR temp
 * @param raw_temp Raw temperature in °C
 * @return Calibrated temperature in °C
 */
float mlx90614_apply_calibration(float raw_temp);

/**
 * @brief Read a 16-bit register word from MLX90614 with PEC CRC validation
 * @param reg Register address (e.g. MLX90614_TOBJ1)
 * @param[out] val Pointer to store 16-bit word
 * @return ESP_OK on success
 */
esp_err_t mlx90614_read_reg16(uint8_t reg, uint16_t *val);

#ifdef __cplusplus
}
#endif

#endif // MLX90614_H
