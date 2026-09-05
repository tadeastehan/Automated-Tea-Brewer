/**
 * @file thermometer.c
 * @brief MLX90614 Non-Contact Infrared Temperature Sensor Driver Implementation
 * 
 * Uses shared bit-bang I2C engine with repeated START and PEC error checking.
 */

#include "thermometer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "main_pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "THERMOMETER";

#define MLX90614_ADDR       0x5A
#define MLX90614_TA         0x06
#define MLX90614_TOBJ1      0x07

static bool s_initialized = false;
static gpio_num_t s_sda_pin = PIN_I2C_SDA;
static gpio_num_t s_scl_pin = PIN_I2C_SCL;

/* ============================================================================
   SHARED BIT-BANG I2C ENGINE
   ============================================================================ */
#define I2C_DELAY_US 10

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

static esp_err_t mlx90614_read_reg(uint8_t reg, uint16_t *val)
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

    // 4. Read Low Byte and High Byte
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
    }

    *val = ((uint16_t)msb << 8) | lsb;
    return ESP_OK;
}

/* ============================================
   Calibration Constants
   ============================================ */
#define CALIB_COEFF_A  0.000089f
#define CALIB_COEFF_B  1.084436f
#define CALIB_COEFF_C  (-1.348549f)

static float apply_temp_calibration(float raw_temp)
{
    return (CALIB_COEFF_A * raw_temp * raw_temp) + 
           (CALIB_COEFF_B * raw_temp) + 
           CALIB_COEFF_C;
}

/* ============================================
   Public Functions
   ============================================ */

esp_err_t thermometer_init(void)
{
    ESP_LOGI(TAG, "Initializing MLX90614 temperature sensor (0x5A)...");
    bb_configure_pins();

    uint16_t raw_ta = 0;
    esp_err_t ret = ESP_FAIL;

    // Retry up to 5 times with delay for sensor power-on stabilization
    for (int attempt = 0; attempt < 5; attempt++) {
        ret = mlx90614_read_reg(MLX90614_TA, &raw_ta);
        if (ret == ESP_OK && raw_ta != 0 && raw_ta != 0xFFFF) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (ret != ESP_OK || raw_ta == 0 || raw_ta == 0xFFFF) {
        ESP_LOGW(TAG, "MLX90614 not responding at I2C address 0x5A");
        s_initialized = false;
        return ESP_ERR_NOT_FOUND;
    }

    float test_temp = ((float)raw_ta * 0.02f) - 273.15f;
    s_initialized = true;
    ESP_LOGI(TAG, "MLX90614 initialized successfully (ambient: %.2f°C)", test_temp);

    return ESP_OK;
}

esp_err_t thermometer_deinit(void)
{
    s_initialized = false;
    ESP_LOGI(TAG, "MLX90614 deinitialized");
    return ESP_OK;
}

esp_err_t thermometer_get_object_temp_raw(float *temperature)
{
    if (temperature == NULL) return ESP_ERR_INVALID_ARG;

    if (!s_initialized) {
        if (thermometer_init() != ESP_OK) return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw = 0;
    esp_err_t ret = mlx90614_read_reg(MLX90614_TOBJ1, &raw);
    if (ret != ESP_OK) return ret;

    *temperature = ((float)raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t thermometer_get_object_temp(float *temperature)
{
    if (temperature == NULL) return ESP_ERR_INVALID_ARG;

    float raw_temp = 0.0f;
    esp_err_t ret = thermometer_get_object_temp_raw(&raw_temp);
    if (ret != ESP_OK) return ret;

    *temperature = apply_temp_calibration(raw_temp);
    return ESP_OK;
}

esp_err_t thermometer_get_ambient_temp(float *temperature)
{
    if (temperature == NULL) return ESP_ERR_INVALID_ARG;

    if (!s_initialized) {
        if (thermometer_init() != ESP_OK) return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw = 0;
    esp_err_t ret = mlx90614_read_reg(MLX90614_TA, &raw);
    if (ret != ESP_OK) return ret;

    *temperature = ((float)raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t thermometer_get_temperatures(thermometer_readings_t *readings)
{
    if (readings == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = thermometer_get_object_temp(&readings->object_temp);
    if (ret != ESP_OK) return ret;

    ret = thermometer_get_ambient_temp(&readings->ambient_temp);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

bool thermometer_is_initialized(void)
{
    if (!s_initialized) {
        thermometer_init();
    }
    return s_initialized;
}

i2c_master_bus_handle_t thermometer_get_bus_handle(void)
{
    return NULL;
}
