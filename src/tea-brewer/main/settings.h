/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "esp_err.h"

#define MAX_TEA_TYPES 6  // Support 6 different tea types

typedef enum {
    LANGUAGE_EN = 0,
    LANGUAGE_CN,
    LANGUAGE_MAX,
} LANGUAGE_SET;

// Tea parameters for each tea type
typedef struct {
    uint8_t temperature;      // Temperature in °C (75-100)
    uint16_t infusion_time;   // Infusion time in seconds (0-899, i.e., 0-14:59)
} tea_params_t;

typedef struct {
    uint8_t magic;
    bool need_hint;
    uint8_t language;
    int idle_position;        // Idle position percentage (0-100)
    tea_params_t tea_params[MAX_TEA_TYPES];  // Tea-specific parameters
    
    // Schedule parameters
    bool schedule_active;
    uint8_t schedule_hour;
    uint8_t schedule_minute;
    uint8_t schedule_target_temp;
    uint8_t schedule_tea_index;
} sys_param_t;

esp_err_t settings_read_parameter_from_nvs(void);
esp_err_t settings_write_parameter_to_nvs(void);
sys_param_t *settings_get_parameter(void);

// Idle position functions
void settings_set_idle_position(int position);
int settings_get_idle_position(void);
void settings_set_idle_position_no_save(int position);  // Set position without saving to NVS
void settings_flush_idle_position(void);  // Force write idle position to NVS

// Schedule functions
void settings_set_schedule(uint8_t hour, uint8_t minute, uint8_t target_temp, uint8_t tea_index);
bool settings_get_schedule(uint8_t *hour, uint8_t *minute, uint8_t *target_temp, uint8_t *tea_index);
void settings_clear_schedule(void);

// Tea parameters functions
void settings_set_tea_temperature(uint8_t tea_index, uint8_t temperature);
uint8_t settings_get_tea_temperature(uint8_t tea_index);
void settings_set_tea_infusion_time(uint8_t tea_index, uint16_t seconds);
uint16_t settings_get_tea_infusion_time(uint8_t tea_index);
void settings_set_tea_temperature_no_save(uint8_t tea_index, uint8_t temperature);
void settings_set_tea_infusion_time_no_save(uint8_t tea_index, uint16_t seconds);
void settings_flush_tea_params(void);  // Force write tea parameters to NVS

