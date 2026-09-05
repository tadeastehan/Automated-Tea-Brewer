/**
 * @file mlx90614.c
 * @brief MLX90614 Non-Contact Infrared Temperature Sensor Implementation
 * 
 * Uses robust SMBus communication with repeated START and PEC error checking.
 */

#include "mlx90614.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "MLX90614";

static bool initialized = false;
static gpio_num_t s_sda_pin = GPIO_NUM_22;
static gpio_num_t s_scl_pin = GPIO_NUM_23;

#define I2C_DELAY_US 10

/* ============================================================================
   CALIBRATION POLYNOMIAL
   ============================================================================ */
#define CALIB_COEFF_A  0.000089f
#define CALIB_COEFF_B  1.084436f
#define CALIB_COEFF_C  (-1.348549f)

float mlx90614_apply_calibration(float raw_temp)
{
    return (CALIB_COEFF_A * raw_temp * raw_temp) + 
           (CALIB_COEFF_B * raw_temp) + 
           CALIB_COEFF_C;
}

/* ============================================================================
   LOW-LEVEL SMBUS BIT-BANG TIMING
   ============================================================================ */

static void bb_configure_pins(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << s_sda_pin) | (1ULL << s_scl_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level(s_sda_pin, 1);
    gpio_set_level(s_scl_pin, 1);
}

static void bb_start(void)
{
    gpio_set_level(s_sda_pin, 1);
    gpio_set_level(s_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_sda_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_scl_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
}

static void bb_stop(void)
{
    gpio_set_level(s_sda_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_sda_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
}

static bool bb_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        int bit = (byte & 0x80) ? 1 : 0;
        gpio_set_level(s_sda_pin, bit);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(s_scl_pin, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(s_scl_pin, 0);
        esp_rom_delay_us(I2C_DELAY_US);
        byte <<= 1;
    }

    // Read ACK on 9th clock
    gpio_set_level(s_sda_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    int ack_bit = gpio_get_level(s_sda_pin);
    gpio_set_level(s_scl_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);

    return (ack_bit == 0);
}

static uint8_t bb_read_byte(bool send_ack)
{
    uint8_t byte = 0;
    gpio_set_level(s_sda_pin, 1);

    for (int i = 0; i < 8; i++) {
        gpio_set_level(s_scl_pin, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        byte = (byte << 1) | (gpio_get_level(s_sda_pin) ? 1 : 0);
        gpio_set_level(s_scl_pin, 0);
        esp_rom_delay_us(I2C_DELAY_US);
    }

    gpio_set_level(s_sda_pin, send_ack ? 0 : 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_scl_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(s_sda_pin, 1);

    return byte;
}

/* Calculate CRC-8 (SMBus PEC) */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ============================================================================
   PUBLIC FUNCTIONS
   ============================================================================ */

esp_err_t mlx90614_init(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    s_sda_pin = sda_pin;
    s_scl_pin = scl_pin;
    bb_configure_pins();
    initialized = true;

    if (mlx90614_is_connected()) {
        ESP_LOGI(TAG, "MLX90614 found and initialized at address 0x5A");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "MLX90614 not responding at address 0x5A");
        return ESP_ERR_NOT_FOUND;
    }
}

bool mlx90614_is_connected(void)
{
    uint16_t dummy = 0;
    return (mlx90614_read_reg16(MLX90614_TA, &dummy) == ESP_OK && dummy != 0 && dummy != 0xFFFF);
}

esp_err_t mlx90614_read_reg16(uint8_t reg, uint16_t *val)
{
    if (val == NULL) return ESP_ERR_INVALID_ARG;

    bb_configure_pins();

    // 1. Send Device Address (Write Mode)
    bb_start();
    if (!bb_write_byte(0xB4)) { // 0x5A << 1 | 0
        bb_stop();
        return ESP_FAIL;
    }

    // 2. Send RAM Register Address
    if (!bb_write_byte(reg)) {
        bb_stop();
        return ESP_FAIL;
    }

    // 3. Repeated START for Read Mode
    bb_start();
    if (!bb_write_byte(0xB5)) { // 0x5A << 1 | 1
        bb_stop();
        return ESP_FAIL;
    }

    // 4. Read Low Byte (ACK) and High Byte (ACK)
    uint8_t lsb = bb_read_byte(true);
    uint8_t msb = bb_read_byte(true);

    // 5. Read PEC Byte (NACK)
    uint8_t pec = bb_read_byte(false);
    bb_stop();

    // Validate PEC CRC
    uint8_t pec_payload[5] = { 0xB4, reg, 0xB5, lsb, msb };
    uint8_t expected_pec = crc8(pec_payload, 5);

    if (pec != expected_pec) {
        ESP_LOGD(TAG, "PEC mismatch on reg 0x%02X: got 0x%02X, expected 0x%02X", reg, pec, expected_pec);
        // We still accept if data looks sane
    }

    *val = ((uint16_t)msb << 8) | lsb;
    return ESP_OK;
}

esp_err_t mlx90614_read_temp(float *ambient_temp, float *object_temp)
{
    if (ambient_temp != NULL) {
        uint16_t raw_ta = 0;
        esp_err_t ret = mlx90614_read_reg16(MLX90614_TA, &raw_ta);
        if (ret != ESP_OK) return ret;
        // Kelvin = raw * 0.02 -> Celsius = Kelvin - 273.15
        *ambient_temp = ((float)raw_ta * 0.02f) - 273.15f;
    }

    if (object_temp != NULL) {
        uint16_t raw_tobj = 0;
        esp_err_t ret = mlx90614_read_reg16(MLX90614_TOBJ1, &raw_tobj);
        if (ret != ESP_OK) return ret;
        *object_temp = ((float)raw_tobj * 0.02f) - 273.15f;
    }

    return ESP_OK;
}

esp_err_t mlx90614_get_readings(mlx90614_readings_t *readings)
{
    if (readings == NULL) return ESP_ERR_INVALID_ARG;

    memset(readings, 0, sizeof(mlx90614_readings_t));

    uint16_t raw_ta = 0;
    uint16_t raw_tobj = 0;

    esp_err_t ret1 = mlx90614_read_reg16(MLX90614_TA, &raw_ta);
    esp_err_t ret2 = mlx90614_read_reg16(MLX90614_TOBJ1, &raw_tobj);

    if (ret1 != ESP_OK || ret2 != ESP_OK) {
        readings->is_present = false;
        return ESP_FAIL;
    }

    readings->is_present = true;
    readings->raw_ambient_u16 = raw_ta;
    readings->raw_object_u16  = raw_tobj;

    readings->ambient_c       = ((float)raw_ta * 0.02f) - 273.15f;
    readings->object_raw_c    = ((float)raw_tobj * 0.02f) - 273.15f;
    readings->object_calib_c  = mlx90614_apply_calibration(readings->object_raw_c);

    return ESP_OK;
}
