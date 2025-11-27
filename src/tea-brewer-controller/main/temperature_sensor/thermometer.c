/**
 * @file thermometer.c
 * @brief MLX90614 Infrared Temperature Sensor Driver Implementation
 * 
 * Uses the larryli/mlx90614 library component.
 * Add dependency: idf.py add-dependency "larryli/mlx90614"
 */

#include "thermometer.h"
#include "driver/i2c_master.h"
#include "mlx90614.h"
#include "esp_log.h"
#include "main_pins.h"

static const char *TAG = "THERMOMETER";

/* ============================================
   Configuration
   ============================================ */
#define MLX90614_I2C_FREQ_HZ    100000  /**< I2C frequency (100kHz) */

/* ============================================
   Static Variables
   ============================================ */
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static mlx90614_handle_t s_mlx90614_handle = NULL;
static bool s_initialized = false;

/* ============================================
   Public Functions
   ============================================ */

esp_err_t thermometer_init(void)
{
    esp_err_t ret;

    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing MLX90614 temperature sensor...");

    /* Configure I2C master bus */
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure MLX90614 using the library */
    mlx90614_config_t mlx90614_config = {
        .mlx90614_device.scl_speed_hz = MLX90614_I2C_FREQ_HZ,
        .mlx90614_device.device_address = 0x5A,
    };

    ret = mlx90614_init(s_i2c_bus_handle, &mlx90614_config, &s_mlx90614_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MLX90614: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
        return ret;
    }

    /* Test communication by reading ambient temperature */
    float test_temp;
    ret = mlx90614_get_ta(s_mlx90614_handle, &test_temp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with MLX90614");
        i2c_del_master_bus(s_i2c_bus_handle);
        s_mlx90614_handle = NULL;
        s_i2c_bus_handle = NULL;
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "MLX90614 initialized successfully (ambient: %.2f°C)", test_temp);

    return ESP_OK;
}

esp_err_t thermometer_deinit(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_mlx90614_handle) {
        mlx90614_deinit(s_mlx90614_handle);
        s_mlx90614_handle = NULL;
    }

    if (s_i2c_bus_handle) {
        i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "MLX90614 deinitialized");

    return ESP_OK;
}

esp_err_t thermometer_get_object_temp(float *temperature)
{
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return mlx90614_get_to(s_mlx90614_handle, temperature);
}

esp_err_t thermometer_get_ambient_temp(float *temperature)
{
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return mlx90614_get_ta(s_mlx90614_handle, temperature);
}

esp_err_t thermometer_get_temperatures(thermometer_readings_t *readings)
{
    if (readings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    ret = mlx90614_get_to(s_mlx90614_handle, &readings->object_temp);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mlx90614_get_ta(s_mlx90614_handle, &readings->ambient_temp);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

bool thermometer_is_initialized(void)
{
    return s_initialized;
}
