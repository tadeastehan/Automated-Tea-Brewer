/**
 * @file pot_sensor.c
 * @brief Tea Pot Detection Module Implementation
 */

#include "pot_sensor.h"
#include "distance_sensor/distance_sensor.h"
#include "esp_log.h"

static const char *TAG = "POT_SENSOR";

/* ============================================
   Static Variables
   ============================================ */
static bool s_initialized = false;
static uint16_t s_threshold_mm = POT_THRESHOLD_DEFAULT_MM;

/* ============================================
   Public Functions
   ============================================ */

esp_err_t pot_sensor_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* Check if distance sensor is available */
    if (!distance_sensor_is_initialized()) {
        ESP_LOGE(TAG, "Distance sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_threshold_mm = POT_THRESHOLD_DEFAULT_MM;
    s_initialized = true;
    
    ESP_LOGI(TAG, "Pot sensor initialized (threshold: %u mm)", s_threshold_mm);
    return ESP_OK;
}

esp_err_t pot_sensor_check(bool *is_present)
{
    if (is_present == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t distance_mm;
    esp_err_t ret = distance_sensor_get_distance(&distance_mm);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read distance: %s", esp_err_to_name(ret));
        *is_present = false;
        return ESP_FAIL;
    }

    *is_present = (distance_mm < s_threshold_mm);
    
    ESP_LOGD(TAG, "Distance: %u mm, Threshold: %u mm, Pot present: %s", 
             distance_mm, s_threshold_mm, *is_present ? "YES" : "NO");

    return ESP_OK;
}

bool pot_sensor_is_present(void)
{
    bool is_present = false;
    
    if (pot_sensor_check(&is_present) != ESP_OK) {
        return false;
    }
    
    return is_present;
}

esp_err_t pot_sensor_get_distance(uint16_t *distance_mm)
{
    if (distance_mm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return distance_sensor_get_distance(distance_mm);
}

void pot_sensor_set_threshold(uint16_t threshold_mm)
{
    s_threshold_mm = threshold_mm;
    ESP_LOGI(TAG, "Threshold set to %u mm", s_threshold_mm);
}

uint16_t pot_sensor_get_threshold(void)
{
    return s_threshold_mm;
}

bool pot_sensor_is_initialized(void)
{
    return s_initialized;
}
