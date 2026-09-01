/**
 * @file thermometer.c
 * @brief MLX90614 Infrared Temperature Sensor Driver Implementation
 * 
 * Uses shared bit-bang I2C engine with repeated START and PEC error checking.
 */

#include "thermometer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "main_pins.h"
#include <string.h>

static const char *TAG = "THERMOMETER";

#define MLX90614_ADDR       0x5A
#define MLX90614_TA         0x06
#define MLX90614_TOBJ1      0x07

static bool s_initialized = false;

/* ============================================================================
   SHARED BIT-BANG I2C ENGINE
   ============================================================================ */
#define I2C_DELAY_US 10

static void bb_configure_pins(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << PIN_I2C_SDA) | (1ULL << PIN_I2C_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level(PIN_I2C_SDA, 1);
    gpio_set_level(PIN_I2C_SCL, 1);
}

static void bb_start(void)
{
    gpio_set_level(PIN_I2C_SDA, 1);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SDA, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(I2C_DELAY_US);
}

static void bb_stop(void)
{
    gpio_set_level(PIN_I2C_SDA, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SDA, 1);
    esp_rom_delay_us(I2C_DELAY_US);
}

static bool bb_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        int bit = (byte & 0x80) ? 1 : 0;
        gpio_set_level(PIN_I2C_SDA, bit);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(PIN_I2C_SCL, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(PIN_I2C_SCL, 0);
        esp_rom_delay_us(I2C_DELAY_US);
        byte <<= 1;
    }

    gpio_set_level(PIN_I2C_SDA, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    int ack_bit = gpio_get_level(PIN_I2C_SDA);
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(I2C_DELAY_US);

    return (ack_bit == 0);
}

static uint8_t bb_read_byte(bool send_ack)
{
    uint8_t byte = 0;
    gpio_set_level(PIN_I2C_SDA, 1);

    for (int i = 0; i < 8; i++) {
        gpio_set_level(PIN_I2C_SCL, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        byte = (byte << 1) | (gpio_get_level(PIN_I2C_SDA) ? 1 : 0);
        gpio_set_level(PIN_I2C_SCL, 0);
        esp_rom_delay_us(I2C_DELAY_US);
    }

    gpio_set_level(PIN_I2C_SDA, send_ack ? 0 : 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SDA, 1);

    return byte;
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
    (void)bb_read_byte(false);
    bb_stop();

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
    ESP_LOGI(TAG, "Initializing MLX90614 temperature sensor...");
    bb_configure_pins();

    uint16_t raw_ta = 0;
    esp_err_t ret = mlx90614_read_reg(MLX90614_TA, &raw_ta);
    if (ret != ESP_OK || raw_ta == 0 || raw_ta == 0xFFFF) {
        ESP_LOGE(TAG, "MLX90614 not responding at I2C address 0x5A");
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
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    uint16_t raw = 0;
    esp_err_t ret = mlx90614_read_reg(MLX90614_TOBJ1, &raw);
    if (ret != ESP_OK) return ret;

    *temperature = ((float)raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t thermometer_get_object_temp(float *temperature)
{
    if (temperature == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    float raw_temp = 0.0f;
    esp_err_t ret = thermometer_get_object_temp_raw(&raw_temp);
    if (ret != ESP_OK) return ret;

    *temperature = apply_temp_calibration(raw_temp);
    return ESP_OK;
}

esp_err_t thermometer_get_ambient_temp(float *temperature)
{
    if (temperature == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    uint16_t raw = 0;
    esp_err_t ret = mlx90614_read_reg(MLX90614_TA, &raw);
    if (ret != ESP_OK) return ret;

    *temperature = ((float)raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t thermometer_get_temperatures(thermometer_readings_t *readings)
{
    if (readings == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = thermometer_get_object_temp(&readings->object_temp);
    if (ret != ESP_OK) return ret;

    ret = thermometer_get_ambient_temp(&readings->ambient_temp);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

bool thermometer_is_initialized(void)
{
    return s_initialized;
}

i2c_master_bus_handle_t thermometer_get_bus_handle(void)
{
    return NULL;
}
