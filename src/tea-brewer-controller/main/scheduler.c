#include "scheduler.h"
#include "rtc/rtc.h"
#include "console/uart_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include <time.h>

static const char *TAG = "SCHEDULER";
static TimerHandle_t scheduler_timer = NULL;
static uint32_t scheduled_target_time = 0;
static uint8_t scheduled_target_temp = 0;
static uint8_t scheduled_brewing_temp = 0;
static bool is_active = false;

// Helper function to calculate time to cool to a specific temperature
// t(T) = -ln((T - 26.9379) / 67.5826) / 0.6109 (hours)
static double calculate_cooling_time_hours(double temp) {
    if (temp <= 27.0) return 0; // Avoid domain error
    return -log((temp - 26.9379) / 67.5826) / 0.6109;
}

// Helper function to calculate brewing time to reach a specific temperature
// t(T) = -ln(1 - (T - 16.8791) / 377.4376) / 0.000259 (seconds)
// Fitted from experimental heating curve data
static uint32_t calculate_brewing_time_sec(double temp) {
    if (temp <= 16.88) return 0; // Below starting temperature
    double arg = 1.0 - (temp - 16.8791) / 377.4376;
    if (arg <= 0) return 16 * 60; // Cap at 16 minutes if beyond model range
    return (uint32_t)(-log(arg) / 0.000259);
}

static void scheduler_check_callback(TimerHandle_t xTimer) {
    if (!is_active) return;

    struct tm timeinfo;
    if (rtc_get_time_with_timezone(&timeinfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get time from RTC");
        return;
    }
    
    time_t now = mktime(&timeinfo);
    
    // 1. Calculate brewing time based on brewing temperature
    uint32_t brewing_time_sec = calculate_brewing_time_sec((double)scheduled_brewing_temp);
    
    // 2. Calculate cooling time
    // Time to cool from Brewing Temp -> Target Temp
    double t_target = calculate_cooling_time_hours((double)scheduled_target_temp);
    double t_brew = calculate_cooling_time_hours((double)scheduled_brewing_temp);
    
    // Duration is the difference in time points on the cooling curve
    // t_target will be larger (later time) than t_brew (earlier time)
    double cooling_time_hours = t_target - t_brew;
    if (cooling_time_hours < 0) cooling_time_hours = 0;
    
    uint32_t cooling_time_sec = (uint32_t)(cooling_time_hours * 3600.0);
    
    // Total lead time required
    uint32_t total_lead_time_sec = brewing_time_sec + cooling_time_sec;
    
    ESP_LOGD(TAG, "Now: %lld, Target: %lu", now, scheduled_target_time);
    ESP_LOGD(TAG, "Brew Time: %lu s, Cool Time: %lu s, Total Lead: %lu s", 
             brewing_time_sec, cooling_time_sec, total_lead_time_sec);

    // Check if time to start
    // We use a 30s buffer to ensure we don't miss it if the timer drifts slightly
    if (now >= (scheduled_target_time - total_lead_time_sec)) {
        // Safety check: if we are already past the target time, we missed it
        if (now > scheduled_target_time) {
            ESP_LOGW(TAG, "Missed scheduled target time! (Now: %lld, Target: %lu)", now, scheduled_target_time);
            is_active = false;
            xTimerStop(scheduler_timer, 0);
            return;
        }

        ESP_LOGI(TAG, "Scheduler triggered! Starting brew.");
        
        // Notify Display
        uart_comm_send_notification(NOTIFY_BREW_STARTED);
        
        // Disable scheduler
        is_active = false;
        xTimerStop(scheduler_timer, 0);
    }
}

void scheduler_init(void) {
    scheduler_timer = xTimerCreate("scheduler_timer", pdMS_TO_TICKS(30000), pdTRUE, NULL, scheduler_check_callback);
}

void scheduler_set(uint8_t hour, uint8_t minute, uint8_t target_temp, uint8_t brewing_temp) {
    struct tm timeinfo;
    if (rtc_get_time_with_timezone(&timeinfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get time from RTC");
        return;
    }
    
    // Create target time struct
    struct tm target_tm = timeinfo;
    target_tm.tm_hour = hour;
    target_tm.tm_min = minute;
    target_tm.tm_sec = 0;
    
    time_t now = mktime(&timeinfo);
    time_t target = mktime(&target_tm);
    
    // If target is in the past, add 24 hours
    bool is_tomorrow = false;
    if (target <= now) {
        target += 24 * 3600;
        is_tomorrow = true;
    }
    
    scheduled_target_time = (uint32_t)target;
    scheduled_target_temp = target_temp;
    scheduled_brewing_temp = brewing_temp;
    is_active = true;
    
    // Calculate lead time for logging
    uint32_t brewing_time_sec = calculate_brewing_time_sec((double)brewing_temp);
    double t_target = calculate_cooling_time_hours((double)target_temp);
    double t_brew = calculate_cooling_time_hours((double)brewing_temp);
    double cooling_time_hours = t_target - t_brew;
    if (cooling_time_hours < 0) cooling_time_hours = 0;
    uint32_t total_lead_time_sec = brewing_time_sec + (uint32_t)(cooling_time_hours * 3600.0);
    
    time_t start_time = target - total_lead_time_sec;
    struct tm *start_tm = localtime(&start_time);
    
    // If calculated start time is in the past, it means we need to schedule for tomorrow
    if (start_time < now) {
        struct tm now_tm = *localtime(&now);
        ESP_LOGW(TAG, "Calculated start time %02d:%02d is in the past (Now: %02d:%02d). Moving schedule to tomorrow.",
                 start_tm->tm_hour, start_tm->tm_min, now_tm.tm_hour, now_tm.tm_min);
        
        target += 24 * 3600;
        is_tomorrow = true;
        
        // Recalculate start time
        start_time = target - total_lead_time_sec;
        start_tm = localtime(&start_time);
    }
    
    ESP_LOGI(TAG, "Schedule set for %02d:%02d %s (Target Temp: %d C)", 
             hour, minute, is_tomorrow ? "TOMORROW" : "TODAY", target_temp);
    ESP_LOGI(TAG, "Brewing will start at %02d:%02d (Lead time: %lu min)", 
             start_tm->tm_hour, start_tm->tm_min, total_lead_time_sec / 60);
    
    if (scheduler_timer) {
        xTimerStart(scheduler_timer, 0);
    }
}

void scheduler_cancel(void) {
    is_active = false;
    if (scheduler_timer) {
        xTimerStop(scheduler_timer, 0);
    }
    ESP_LOGI(TAG, "Schedule cancelled");
}

bool scheduler_is_active(void) {
    return is_active;
}

void scheduler_get_status(uint32_t *target_time, uint32_t *remaining_sec) {
    if (!is_active) {
        if (target_time) *target_time = 0;
        if (remaining_sec) *remaining_sec = 0;
        return;
    }
    
    if (target_time) *target_time = scheduled_target_time;
    
    if (remaining_sec) {
        struct tm timeinfo;
        if (rtc_get_time_with_timezone(&timeinfo) == ESP_OK) {
            time_t now = mktime(&timeinfo);
            
            // Calculate lead time again
            uint32_t brewing_time_sec = calculate_brewing_time_sec((double)scheduled_brewing_temp);
            double t_target = calculate_cooling_time_hours((double)scheduled_target_temp);
            double t_brew = calculate_cooling_time_hours((double)scheduled_brewing_temp);
            double cooling_time_hours = t_target - t_brew;
            if (cooling_time_hours < 0) cooling_time_hours = 0;
            uint32_t total_lead_time_sec = brewing_time_sec + (uint32_t)(cooling_time_hours * 3600.0);
            
            time_t start_time = scheduled_target_time - total_lead_time_sec;
            
            if (start_time > now) {
                *remaining_sec = (uint32_t)(start_time - now);
            } else {
                *remaining_sec = 0;
            }
        } else {
            *remaining_sec = 0;
        }
    }
}
