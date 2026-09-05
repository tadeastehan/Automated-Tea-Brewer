#include "console.h"
#include "../motor/motor_control.h"
#include "../temperature_sensor/thermometer.h"
#include "../pot_sensor/pot_sensor.h"
#include "../rtc/rtc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#define RX_BUF_SIZE 256

/* ============================================
   USB SERIAL FUNCTIONS
   ============================================ */
esp_err_t console_init(void)
{
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = RX_BUF_SIZE,
    };
    return usb_serial_jtag_driver_install(&config);
}

void console_print(const char *str)
{
    usb_serial_jtag_write_bytes((const uint8_t *)str, strlen(str), pdMS_TO_TICKS(100));
}

void console_printf(const char *fmt, ...)
{
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    console_print(buf);
}

static int console_getchar(void)
{
    uint8_t c;
    int len = usb_serial_jtag_read_bytes(&c, 1, 0);
    return (len > 0) ? (int)c : -1;
}

bool console_has_input(void)
{
    uint8_t c;
    int len = usb_serial_jtag_read_bytes(&c, 1, 0);
    if (len > 0) {
        // Put it back (we just peeked)
        // Note: This is a simple implementation - proper peek would need buffering
        return true;
    }
    return false;
}

/* ============================================
   COMMAND HANDLERS
   ============================================ */
static void print_help(void)
{
    console_printf("\r\n=== TEA BREWER CONTROLLER COMMANDS ===\r\n");
    console_printf("  c     - Full calibration (without load)\r\n");
    console_printf("  t     - Calibrate with SG monitoring\r\n");
    console_printf("  f     - Fast home\r\n");
    console_printf("  0-100 - Move to percent\r\n");
    console_printf("  r     - Go to 0%%\r\n");
    console_printf("  m     - Go to 50%%\r\n");
    console_printf("  g     - Read StallGuard value\r\n");
    console_printf("  i/d   - Increase/decrease SGT\r\n");
    console_printf("  w     - Save to flash\r\n");
    console_printf("  z     - Clear calibration\r\n");
    console_printf("  s     - Status (Motor, Temp, Dist, RTC)\r\n");
    console_printf("  temp  - Read MLX90614 IR & ambient temperature\r\n");
    console_printf("  dist  - Read VL53L0X laser distance sensor\r\n");
    console_printf("  e/x   - Enable/disable motor\r\n");
    console_printf("  h     - Help\r\n");
    console_printf("======================================\r\n\r\n");
}

static void print_status(void)
{
    motor_status_t status;
    motor_get_status(&status);
    
    console_printf("\r\n================ SYSTEM STATUS ================\r\n");
    console_printf(" [STEPPER MOTOR]\r\n");
    console_printf("  Calibrated : %s\r\n", status.is_calibrated ? "YES" : "NO");
    console_printf("  Homed      : %s\r\n", status.is_homed ? "YES" : "NO");
    console_printf("  Position   : %ld steps (%.1f%%)\r\n", 
                   (long)status.position_steps, status.position_percent);
    console_printf("  Total Range: %ld steps\r\n", (long)status.total_steps);
    console_printf("  SGT Thresh : %d\r\n", status.sgt_threshold);

    console_printf("\r\n [MLX90614 IR THERMOMETER]\r\n");
    if (thermometer_is_initialized()) {
        float obj_temp = 0.0f, amb_temp = 0.0f, raw_obj = 0.0f;
        thermometer_get_object_temp(&obj_temp);
        thermometer_get_object_temp_raw(&raw_obj);
        thermometer_get_ambient_temp(&amb_temp);
        console_printf("  Object (Calibrated) : %.1f °C (%.1f °F)\r\n", obj_temp, (obj_temp * 1.8f) + 32.0f);
        console_printf("  Object (Raw IR)     : %.1f °C (%.1f °F)\r\n", raw_obj, (raw_obj * 1.8f) + 32.0f);
        console_printf("  Ambient Temp        : %.1f °C (%.1f °F)\r\n", amb_temp, (amb_temp * 1.8f) + 32.0f);
    } else {
        console_printf("  Status: NOT INITIALIZED / OFFLINE\r\n");
    }

    console_printf("\r\n [VL53L0X DISTANCE / POT SENSOR]\r\n");
    if (pot_sensor_is_initialized()) {
        uint16_t dist_mm = 0;
        pot_sensor_get_distance(&dist_mm);
        bool present = pot_sensor_is_present();
        console_printf("  Distance : %u mm (%.1f cm)\r\n", dist_mm, (float)dist_mm / 10.0f);
        console_printf("  Pot State: %s (Threshold: %u mm)\r\n", 
                       present ? "PRESENT [OK]" : "NOT PRESENT", pot_sensor_get_threshold());
    } else {
        console_printf("  Status: NOT INITIALIZED / OFFLINE\r\n");
    }

    console_printf("\r\n [D8563TS REAL-TIME CLOCK]\r\n");
    if (rtc_is_initialized()) {
        struct tm timeinfo;
        if (rtc_get_time_with_timezone(&timeinfo) == ESP_OK) {
            console_printf("  Time: %04d-%02d-%02d %02d:%02d:%02d (GMT+1)\r\n",
                           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        } else {
            console_printf("  Time: READ ERROR\r\n");
        }
    } else {
        console_printf("  Status: NOT INITIALIZED / OFFLINE\r\n");
    }
    console_printf("===============================================\r\n\r\n");
}

static void process_command(const char *cmd)
{
    if (strlen(cmd) == 0) return;

    if (strcmp(cmd, "temp") == 0 || strcmp(cmd, "ir") == 0) {
        if (thermometer_is_initialized()) {
            float obj_temp = 0.0f, amb_temp = 0.0f, raw_obj = 0.0f;
            thermometer_get_object_temp(&obj_temp);
            thermometer_get_object_temp_raw(&raw_obj);
            thermometer_get_ambient_temp(&amb_temp);
            console_printf("\r\n--- MLX90614 Temperature Reading ---\r\n");
            console_printf("  Object (Calibrated) : %.1f °C (%.1f °F)\r\n", obj_temp, (obj_temp * 1.8f) + 32.0f);
            console_printf("  Object (Raw IR)     : %.1f °C (%.1f °F)\r\n", raw_obj, (raw_obj * 1.8f) + 32.0f);
            console_printf("  Ambient Temp        : %.1f °C (%.1f °F)\r\n", amb_temp, (amb_temp * 1.8f) + 32.0f);
            console_printf("------------------------------------\r\n\r\n");
        } else {
            console_printf("Thermometer not initialized!\r\n");
        }
        return;
    }

    if (strcmp(cmd, "dist") == 0 || strcmp(cmd, "tof") == 0) {
        if (pot_sensor_is_initialized()) {
            uint16_t dist_mm = 0;
            pot_sensor_get_distance(&dist_mm);
            bool present = pot_sensor_is_present();
            console_printf("\r\n--- VL53L0X Distance Reading ---\r\n");
            console_printf("  Distance : %u mm (%.1f cm)\r\n", dist_mm, (float)dist_mm / 10.0f);
            console_printf("  Pot State: %s\r\n", present ? "PRESENT" : "NOT PRESENT");
            console_printf("--------------------------------\r\n\r\n");
        } else {
            console_printf("Distance sensor not initialized!\r\n");
        }
        return;
    }
    
    if (strlen(cmd) == 1) {
        switch (cmd[0]) {
            case 'c': case 'C':
                console_printf("Starting calibration...\r\n");
                motor_calibrate();
                console_printf("Calibration complete.\r\n");
                break;
            case 't': case 'T':
                console_printf("Starting calibration with SG monitoring...\r\n");
                console_printf("Press any key to stop monitoring.\r\n\r\n");
                motor_calibrate_with_sg_monitor();
                console_printf("Calibration complete.\r\n");
                break;
            case 'g': case 'G':
                {
                    uint16_t sg_value = motor_read_stallguard();
                    console_printf("StallGuard value: %u\r\n", sg_value);
                }
                break;
            case 'f': case 'F':
                console_printf("Homing...\r\n");
                if (motor_home() == ESP_OK) {
                    console_printf("Homing complete.\r\n");
                } else {
                    console_printf("Homing failed - not calibrated?\r\n");
                }
                break;
            case 'i': case 'I':
                console_printf("SGT: %d\r\n", motor_increase_sgt());
                break;
            case 'd': case 'D':
                console_printf("SGT: %d\r\n", motor_decrease_sgt());
                break;
            case 's': case 'S':
                print_status();
                break;
            case 'h': case 'H': case '?':
                print_help();
                break;
            case 'r': case 'R':
                motor_move_to_percent(0.0f);
                break;
            case 'm': case 'M':
                motor_move_to_percent(50.0f);
                break;
            case 'e': case 'E':
                motor_enable();
                console_printf("Motor enabled.\r\n");
                break;
            case 'x': case 'X':
                motor_disable();
                console_printf("Motor STOPPED.\r\n");
                break;
            case 'w': case 'W':
                motor_save_calibration();
                console_printf("Saved.\r\n");
                break;
            case 'z': case 'Z':
                motor_clear_calibration();
                console_printf("Calibration cleared.\r\n");
                break;
            default:
                if (cmd[0] >= '0' && cmd[0] <= '9') {
                    motor_move_to_percent((float)(cmd[0] - '0') * 10.0f);
                } else {
                    console_printf("Unknown: %s\r\n", cmd);
                }
                break;
        }
        return;
    }
    
    /* Multi-character - parse as number */
    char *endptr;
    float value = strtof(cmd, &endptr);
    if (endptr != cmd && (*endptr == '\0' || *endptr == '\r' || *endptr == '\n')) {
        if (value >= 0.0f && value <= 100.0f) {
            motor_move_to_percent(value);
        } else {
            console_printf("Value must be 0-100\r\n");
        }
    } else {
        console_printf("Unknown: %s\r\n", cmd);
    }
}

/* ============================================
   CONSOLE TASK
   ============================================ */
static void console_task(void *pvParameters)
{
    char cmd_buffer[64];
    int cmd_index = 0;
    
    console_printf("> ");
    
    while (1) {
        int c = console_getchar();
        
        if (c >= 0) {
            if (c == '\r' || c == '\n') {
                console_printf("\r\n");
                cmd_buffer[cmd_index] = '\0';
                if (cmd_index > 0) process_command(cmd_buffer);
                cmd_index = 0;
                console_printf("> ");
            } else if (c == 127 || c == 8) {
                if (cmd_index > 0) {
                    cmd_index--;
                    console_printf("\b \b");
                }
            } else if (c >= 32 && c < 127 && cmd_index < (int)sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_index++] = (char)c;
                char echo[2] = {(char)c, '\0'};
                console_print(echo);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void console_start_task(void)
{
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
}