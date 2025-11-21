/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bsp/esp-bsp.h"
#include "settings.h"

static const char *TAG = "settings";

#define NAME_SPACE      "sys_param"
#define KEY             "param"
#define MAGIC_HEAD      0xAA

static sys_param_t g_sys_param = {0};

static const sys_param_t g_default_sys_param = {
    .magic = MAGIC_HEAD,
    .need_hint = 1,
    .language = LANGUAGE_EN,
    .drying_time = 10,       // Default 10 minutes
    .drying_position = 0,    // Default position 0
    .tea_params = {
        {.temperature = 80, .infusion_time = 120},   // Tea 0: Green Tea - 2 min, 80°C
        {.temperature = 100, .infusion_time = 240},  // Tea 1: Black Tea - 4 min, 100°C
        {.temperature = 100, .infusion_time = 360},  // Tea 2: Herbal Tea - 6 min, 100°C
        {.temperature = 100, .infusion_time = 360},  // Tea 3: Fruit Tea - 6 min, 100°C
        {.temperature = 80, .infusion_time = 240},   // Tea 4: White Tea - 4 min, 80°C
        {.temperature = 100, .infusion_time = 300},  // Tea 5: Functional Tea - 5 min, 100°C
    },
};

static esp_err_t settings_check(sys_param_t *param)
{
    esp_err_t ret;
    ESP_GOTO_ON_FALSE(param->magic == MAGIC_HEAD, ESP_ERR_INVALID_ARG, reset, TAG, "magic incorrect");
    ESP_GOTO_ON_FALSE(param->language < LANGUAGE_MAX, ESP_ERR_INVALID_ARG, reset, TAG, "language incorrect");
    return ret;
reset:
    ESP_LOGW(TAG, "Set to default");
    memcpy(&g_sys_param, &g_default_sys_param, sizeof(sys_param_t));
    return ret;
}

esp_err_t settings_read_parameter_from_nvs(void)
{
    nvs_handle_t my_handle = 0;
    esp_err_t ret = nvs_open(NAME_SPACE, NVS_READONLY, &my_handle);
    if (ESP_ERR_NVS_NOT_FOUND == ret) {
        ESP_LOGW(TAG, "Not found, Set to default");
        memcpy(&g_sys_param, &g_default_sys_param, sizeof(sys_param_t));
        settings_write_parameter_to_nvs();
        return ESP_OK;
    }

    ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "nvs open failed (0x%x)", ret);

    size_t len = sizeof(sys_param_t);
    ret = nvs_get_blob(my_handle, KEY, &g_sys_param, &len);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "can't read param");
    nvs_close(my_handle);

    settings_check(&g_sys_param);
    return ret;
err:
    if (my_handle) {
        nvs_close(my_handle);
    }
    return ret;
}

esp_err_t settings_write_parameter_to_nvs(void)
{
    ESP_LOGI(TAG, "Saving settings");
    settings_check(&g_sys_param);
    nvs_handle_t my_handle = {0};
    esp_err_t err = nvs_open(NAME_SPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_set_blob(my_handle, KEY, &g_sys_param, sizeof(sys_param_t));
        err |= nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    return ESP_OK == err ? ESP_OK : ESP_FAIL;
}

sys_param_t *settings_get_parameter(void)
{
    return &g_sys_param;
}

void settings_set_drying_time(uint8_t time)
{
    if (time <= 59) {  // Validate range (0-59 minutes)
        g_sys_param.drying_time = time;
        settings_write_parameter_to_nvs();
    }
}

uint8_t settings_get_drying_time(void)
{
    return g_sys_param.drying_time;
}

void settings_set_drying_position(int position)
{
    g_sys_param.drying_position = position;
    settings_write_parameter_to_nvs();
}

int settings_get_drying_position(void)
{
    return g_sys_param.drying_position;
}

void settings_set_drying_position_no_save(int position)
{
    g_sys_param.drying_position = position;
}

void settings_flush_drying_position(void)
{
    settings_write_parameter_to_nvs();
}

void settings_set_tea_temperature(uint8_t tea_index, uint8_t temperature)
{
    if (tea_index < MAX_TEA_TYPES && temperature >= 75 && temperature <= 100) {
        g_sys_param.tea_params[tea_index].temperature = temperature;
        settings_write_parameter_to_nvs();
    }
}

uint8_t settings_get_tea_temperature(uint8_t tea_index)
{
    if (tea_index < MAX_TEA_TYPES) {
        return g_sys_param.tea_params[tea_index].temperature;
    }
    return 100;  // Default fallback
}

void settings_set_tea_infusion_time(uint8_t tea_index, uint16_t seconds)
{
    if (tea_index < MAX_TEA_TYPES && seconds < 900) {  // Max 14:59
        g_sys_param.tea_params[tea_index].infusion_time = seconds;
        settings_write_parameter_to_nvs();
    }
}

uint16_t settings_get_tea_infusion_time(uint8_t tea_index)
{
    if (tea_index < MAX_TEA_TYPES) {
        return g_sys_param.tea_params[tea_index].infusion_time;
    }
    return 180;  // Default 3 minutes
}

void settings_set_tea_temperature_no_save(uint8_t tea_index, uint8_t temperature)
{
    if (tea_index < MAX_TEA_TYPES && temperature >= 75 && temperature <= 100) {
        g_sys_param.tea_params[tea_index].temperature = temperature;
    }
}

void settings_set_tea_infusion_time_no_save(uint8_t tea_index, uint16_t seconds)
{
    if (tea_index < MAX_TEA_TYPES && seconds < 900) {  // Max 14:59
        g_sys_param.tea_params[tea_index].infusion_time = seconds;
    }
}

void settings_flush_tea_params(void)
{
    settings_write_parameter_to_nvs();
}
