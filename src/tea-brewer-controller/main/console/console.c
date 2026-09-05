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
    console_printf("  c            - Full calibration (without load)\r\n");
    console_printf("  t            - Calibrate with SG monitoring\r\n");
    console_printf("  f            - Fast home\r\n");
    console_printf("  move <0-110> - Move to percent (supports up to 110%%, e.g. move 110)\r\n");
    console_printf("  move max     - Move to maximum position (110.0%%)\r\n");
    console_printf("  move min     - Move to minimum position (0.0%%)\r\n");
    console_printf("  0-110        - Direct percentage move (e.g. 110, 50, 0)\r\n");
    console_printf("  r            - Go to 0%% (minimum)\r\n");
    console_printf("  m            - Go to 50%%\r\n");
    console_printf("  max          - Go to 110%% (maximum)\r\n");
    console_printf("  g            - Read StallGuard value\r\n");
    console_printf("  i/d          - Increase/decrease SGT\r\n");
    console_printf("  w            - Save calibration to flash\r\n");
    console_printf("  z            - Clear calibration\r\n");
    console_printf("  s            - Status (Motor, Temp, Dist, RTC, Dropoff)\r\n");
    console_printf("  temp         - Read MLX90614 IR & ambient temperature\r\n");
    console_printf("  dist         - Read VL53L0X laser distance sensor\r\n");
    console_printf("  e/x          - Enable/disable motor\r\n");
    console_printf("  drop         - Execute teabag dropoff sequence immediately\r\n");
    console_printf("  dropoff [help|start|set|cycles|low|high|delay|speed|save|reset]\r\n");
    console_printf("               - View or configure teabag dropoff parameters\r\n");
    console_printf("  speed <10-800> - Set motor speed in RPM (or view current)\r\n");
    console_printf("  cycles <1-100> - Set dropoff shake cycles\r\n");
    console_printf("  history      - View command history (or use UP/DOWN arrows)\r\n");
    console_printf("  h            - Help\r\n");
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

    console_printf("\r\n [TEABAG DROPOFF CONFIGURATION]\r\n");
    motor_dropoff_config_t dcfg;
    motor_get_dropoff_config(&dcfg);
    console_printf("  Cycles : %d\r\n", dcfg.cycles);
    console_printf("  Range  : %.1f%% -> %.1f%%\r\n", dcfg.low_percent, dcfg.high_percent);
    console_printf("  Delay  : %u ms\r\n", dcfg.delay_ms);
    console_printf("  Speed  : %u RPM\r\n", dcfg.speed_rpm);

    console_printf("\r\n [MLX90614 IR THERMOMETER]\r\n");
    if (thermometer_is_initialized()) {
        float obj_temp = 0.0f, amb_temp = 0.0f, raw_obj = 0.0f;
        thermometer_get_object_temp(&obj_temp);
        thermometer_get_object_temp_raw(&raw_obj);
        console_printf("  Object (Calibrated) : %.1f °C (%.1f °F)\r\n", obj_temp, (obj_temp * 1.8f) + 32.0f);
        console_printf("  Object (Raw IR)     : %.1f °C (%.1f °F)\r\n", raw_obj, (raw_obj * 1.8f) + 32.0f);
        console_printf("  Ambient Temp        : %.1f °C (%.1f °F)\r\n", amb_temp, (amb_temp * 1.8f) + 32.0f);
    } else {
        console_printf("  Status: NOT INITIALIZED / OFFLINE\r\n");
    }

    console_printf("\r\n [VL53L0X DISTANCE SENSOR]\r\n");
    if (pot_sensor_is_initialized()) {
        uint16_t dist_mm = 0;
        pot_sensor_get_distance(&dist_mm);
        bool present = pot_sensor_is_present();
        console_printf("  Distance : %u mm (%.1f cm)\r\n", dist_mm, (float)dist_mm / 10.0f);
        console_printf("  Pot State: %s\r\n", present ? "PRESENT" : "NOT PRESENT");
    } else {
        console_printf("  Status: NOT INITIALIZED / OFFLINE\r\n");
    }

    console_printf("\r\n [DS3231 RTC]\r\n");
    if (rtc_is_initialized()) {
        struct tm timeinfo;
        if (rtc_get_time(&timeinfo) == ESP_OK) {
            char strftime_buf[64];
            strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
            console_printf("  Time  : %s\r\n", strftime_buf);
        } else {
            console_printf("  Status: ERROR READING TIME\r\n");
        }
    } else {
        console_printf("  Status: NOT INITIALIZED / OFFLINE\r\n");
    }
    console_printf("===============================================\r\n\r\n");
}

static void print_dropoff_help(void)
{
    motor_dropoff_config_t cfg;
    motor_get_dropoff_config(&cfg);
    console_printf("\r\n=== TEABAG DROPOFF CONFIGURATION ===\r\n");
    console_printf("  Cycles : %d\r\n", cfg.cycles);
    console_printf("  Range  : %.1f%% -> %.1f%%\r\n", cfg.low_percent, cfg.high_percent);
    console_printf("  Delay  : %u ms\r\n", cfg.delay_ms);
    console_printf("  Speed  : %u RPM\r\n", cfg.speed_rpm);
    console_printf("------------------------------------\r\n");
    console_printf("Commands:\r\n");
    console_printf("  dropoff start (or 'drop')   - Run dropoff sequence now\r\n");
    console_printf("  dropoff set <c> <l> <h> <d> [s] - Set all parameters\r\n");
    console_printf("  dropoff cycles <N>          - Set shake cycles (1-100)\r\n");
    console_printf("  dropoff low <%%>             - Set lower position (0-110%%)\r\n");
    console_printf("  dropoff high <%%>            - Set upper position (0-110%%)\r\n");
    console_printf("  dropoff delay <ms>          - Set delay between shakes\r\n");
    console_printf("  dropoff speed <rpm>         - Set motor speed (10-800 RPM)\r\n");
    console_printf("  dropoff save                - Save config to NVS flash\r\n");
    console_printf("  dropoff reset               - Reset to default parameters\r\n");
    console_printf("====================================\r\n\r\n");
}

static void handle_dropoff_command(const char *args)
{
    while (*args == ' ') args++;
    
    if (*args == '\0' || strcmp(args, "status") == 0 || strcmp(args, "help") == 0) {
        print_dropoff_help();
        return;
    }
    
    if (strcmp(args, "start") == 0 || strcmp(args, "test") == 0 || strcmp(args, "run") == 0) {
        motor_dropoff_config_t cfg;
        motor_get_dropoff_config(&cfg);
        console_printf("Executing dropoff (%d cycles, %.1f%% -> %.1f%%, %d ms, %d RPM)...\r\n",
                       cfg.cycles, cfg.low_percent, cfg.high_percent, cfg.delay_ms, cfg.speed_rpm);
        esp_err_t err = motor_execute_configured_dropoff();
        if (err == ESP_OK) {
            console_printf("Dropoff sequence completed successfully.\r\n");
        } else {
            console_printf("Dropoff failed: %s (ensure motor is homed and calibrated!)\r\n", esp_err_to_name(err));
        }
        return;
    }

    if (strncmp(args, "set ", 4) == 0) {
        int cycles = 0;
        float low = 0.0f, high = 0.0f;
        int delay = 0;
        int speed = 0;
        int n = sscanf(args + 4, "%d %f %f %d %d", &cycles, &low, &high, &delay, &speed);
        if (n >= 4) {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            if (cycles < 1 || cycles > 100) {
                console_printf("Invalid cycles (must be 1-100): %d\r\n", cycles);
                return;
            }
            if (low < 0.0f || low > 110.0f || high < 0.0f || high > 110.0f) {
                console_printf("Invalid percentage (must be 0.0-110.0): low=%.1f high=%.1f\r\n", low, high);
                return;
            }
            if (delay < 0 || delay > 10000) {
                console_printf("Invalid delay (must be 0-10000 ms): %d\r\n", delay);
                return;
            }
            cfg.cycles = (uint8_t)cycles;
            cfg.low_percent = low;
            cfg.high_percent = high;
            cfg.delay_ms = (uint16_t)delay;
            if (n >= 5) {
                if (speed < 10 || speed > 800) {
                    console_printf("Invalid speed (must be 10-800 RPM): %d\r\n", speed);
                    return;
                }
                cfg.speed_rpm = (uint16_t)speed;
            }
            motor_set_dropoff_config(&cfg);
            console_printf("Dropoff config updated: %d cycles, %.1f%% -> %.1f%%, %d ms, %d RPM\r\n",
                           cfg.cycles, cfg.low_percent, cfg.high_percent, cfg.delay_ms, cfg.speed_rpm);
            console_printf("Type 'dropoff save' to persist to flash, or 'drop' to test.\r\n");
        } else {
            console_printf("Usage: dropoff set <cycles> <low_%%> <high_%%> <delay_ms> [speed_rpm]\r\n");
            console_printf("Example: dropoff set 3 98.0 101.0 0 180\r\n");
        }
        return;
    }

    if (strncmp(args, "cycles ", 7) == 0) {
        int cycles = atoi(args + 7);
        if (cycles >= 1 && cycles <= 100) {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            cfg.cycles = (uint8_t)cycles;
            motor_set_dropoff_config(&cfg);
            console_printf("Dropoff cycles set to %d. (Type 'dropoff save' to persist)\r\n", cfg.cycles);
        } else {
            console_printf("Invalid cycles (must be 1-100)\r\n");
        }
        return;
    }

    if (strncmp(args, "low ", 4) == 0) {
        float low = strtof(args + 4, NULL);
        if (low >= 0.0f && low <= 110.0f) {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            cfg.low_percent = low;
            motor_set_dropoff_config(&cfg);
            console_printf("Dropoff low position set to %.1f%%. (Type 'dropoff save' to persist)\r\n", cfg.low_percent);
        } else {
            console_printf("Invalid low percentage (must be 0.0 - 110.0)\r\n");
        }
        return;
    }

    if (strncmp(args, "high ", 5) == 0) {
        float high = strtof(args + 5, NULL);
        if (high >= 0.0f && high <= 110.0f) {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            cfg.high_percent = high;
            motor_set_dropoff_config(&cfg);
            console_printf("Dropoff high position set to %.1f%%. (Type 'dropoff save' to persist)\r\n", cfg.high_percent);
        } else {
            console_printf("Invalid high percentage (must be 0.0 - 110.0)\r\n");
        }
        return;
    }

    if (strncmp(args, "delay ", 6) == 0) {
        int delay = atoi(args + 6);
        if (delay >= 0 && delay <= 10000) {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            cfg.delay_ms = (uint16_t)delay;
            motor_set_dropoff_config(&cfg);
            console_printf("Dropoff delay set to %d ms. (Type 'dropoff save' to persist)\r\n", cfg.delay_ms);
        } else {
            console_printf("Invalid delay (must be 0-10000 ms)\r\n");
        }
        return;
    }

    if (strncmp(args, "speed ", 6) == 0) {
        int speed = atoi(args + 6);
        if (speed >= 10 && speed <= 800) {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            cfg.speed_rpm = (uint16_t)speed;
            motor_set_dropoff_config(&cfg);
            console_printf("Dropoff speed set to %d RPM. (Type 'dropoff save' to persist)\r\n", cfg.speed_rpm);
        } else {
            console_printf("Invalid speed (must be 10-800 RPM)\r\n");
        }
        return;
    }

    if (strcmp(args, "save") == 0) {
        if (motor_save_dropoff_config() == ESP_OK) {
            console_printf("Dropoff configuration saved to NVS flash.\r\n");
        } else {
            console_printf("Failed to save dropoff configuration to NVS flash!\r\n");
        }
        return;
    }

    if (strcmp(args, "reset") == 0) {
        motor_reset_dropoff_config();
        motor_save_dropoff_config();
        console_printf("Dropoff configuration reset to defaults and saved.\r\n");
        print_dropoff_help();
        return;
    }

    console_printf("Unknown dropoff subcommand: '%s'. Type 'dropoff help' for usage.\r\n", args);
}

/* ============================================
   COMMAND HISTORY
   ============================================ */
#define HISTORY_MAX 20

static char s_history[HISTORY_MAX][128];
static int s_history_count = 0;
static int s_history_pos = 0;
static char s_temp_buffer[128] = {0};

static void history_add(const char *cmd)
{
    if (cmd == NULL || strlen(cmd) == 0) return;

    // Do not save duplicate of the most recent command
    if (s_history_count > 0 && strcmp(s_history[s_history_count - 1], cmd) == 0) {
        s_history_pos = s_history_count;
        return;
    }

    if (s_history_count < HISTORY_MAX) {
        strncpy(s_history[s_history_count], cmd, sizeof(s_history[0]) - 1);
        s_history[s_history_count][sizeof(s_history[0]) - 1] = '\0';
        s_history_count++;
    } else {
        for (int i = 0; i < HISTORY_MAX - 1; i++) {
            strcpy(s_history[i], s_history[i + 1]);
        }
        strncpy(s_history[HISTORY_MAX - 1], cmd, sizeof(s_history[0]) - 1);
        s_history[HISTORY_MAX - 1][sizeof(s_history[0]) - 1] = '\0';
    }
    s_history_pos = s_history_count;
}

static void print_history(void)
{
    console_printf("\r\n=== COMMAND HISTORY ===\r\n");
    if (s_history_count == 0) {
        console_printf("  (No commands in history)\r\n");
    } else {
        for (int i = 0; i < s_history_count; i++) {
            console_printf("  %2d: %s\r\n", i + 1, s_history[i]);
        }
    }
    console_printf("=======================\r\n\r\n");
}

static void clear_current_line(char *cmd_buffer, int *cursor_pos, int *cmd_len)
{
    // Move cursor to end of current text so backspaces erase everything
    while (*cursor_pos < *cmd_len) {
        console_print("\033[C");
        (*cursor_pos)++;
    }
    // Erase characters backward to prompt
    while (*cmd_len > 0) {
        console_print("\b \b");
        (*cmd_len)--;
    }
    *cursor_pos = 0;
    cmd_buffer[0] = '\0';
}

static void history_up(char *cmd_buffer, int *cursor_pos, int *cmd_len, size_t buffer_size)
{
    if (s_history_count == 0) return;

    if (s_history_pos == s_history_count) {
        // Save current input before navigating
        cmd_buffer[*cmd_len] = '\0';
        strncpy(s_temp_buffer, cmd_buffer, sizeof(s_temp_buffer) - 1);
        s_temp_buffer[sizeof(s_temp_buffer) - 1] = '\0';
    }

    if (s_history_pos > 0) {
        s_history_pos--;

        clear_current_line(cmd_buffer, cursor_pos, cmd_len);

        strncpy(cmd_buffer, s_history[s_history_pos], buffer_size - 1);
        cmd_buffer[buffer_size - 1] = '\0';
        *cmd_len = (int)strlen(cmd_buffer);
        *cursor_pos = *cmd_len;

        console_print(cmd_buffer);
    }
}

static void history_down(char *cmd_buffer, int *cursor_pos, int *cmd_len, size_t buffer_size)
{
    if (s_history_count == 0) return;

    if (s_history_pos < s_history_count) {
        s_history_pos++;

        clear_current_line(cmd_buffer, cursor_pos, cmd_len);

        if (s_history_pos == s_history_count) {
            strncpy(cmd_buffer, s_temp_buffer, buffer_size - 1);
            cmd_buffer[buffer_size - 1] = '\0';
        } else {
            strncpy(cmd_buffer, s_history[s_history_pos], buffer_size - 1);
            cmd_buffer[buffer_size - 1] = '\0';
        }

        *cmd_len = (int)strlen(cmd_buffer);
        *cursor_pos = *cmd_len;

        console_print(cmd_buffer);
    }
}

static void cursor_left(int *cursor_pos)
{
    if (*cursor_pos > 0) {
        (*cursor_pos)--;
        console_print("\b");
    }
}

static void cursor_right(int *cursor_pos, int cmd_len)
{
    if (*cursor_pos < cmd_len) {
        (*cursor_pos)++;
        console_print("\033[C");
    }
}

static void cursor_home(int *cursor_pos)
{
    while (*cursor_pos > 0) {
        console_print("\b");
        (*cursor_pos)--;
    }
}

static void cursor_end(int *cursor_pos, int cmd_len)
{
    while (*cursor_pos < cmd_len) {
        console_print("\033[C");
        (*cursor_pos)++;
    }
}

static void handle_backspace(char *cmd_buffer, int *cursor_pos, int *cmd_len)
{
    if (*cursor_pos > 0) {
        memmove(&cmd_buffer[*cursor_pos - 1], &cmd_buffer[*cursor_pos], *cmd_len - *cursor_pos + 1);
        (*cursor_pos)--;
        (*cmd_len)--;

        console_print("\b");
        console_print(&cmd_buffer[*cursor_pos]);
        console_print(" ");
        int back = (*cmd_len - *cursor_pos) + 1;
        for (int i = 0; i < back; i++) {
            console_print("\b");
        }
    }
}

static void handle_delete(char *cmd_buffer, int *cursor_pos, int *cmd_len)
{
    if (*cursor_pos < *cmd_len) {
        memmove(&cmd_buffer[*cursor_pos], &cmd_buffer[*cursor_pos + 1], *cmd_len - *cursor_pos);
        (*cmd_len)--;

        console_print(&cmd_buffer[*cursor_pos]);
        console_print(" ");
        int back = (*cmd_len - *cursor_pos) + 1;
        for (int i = 0; i < back; i++) {
            console_print("\b");
        }
    }
}

static void handle_insert_char(char *cmd_buffer, int *cursor_pos, int *cmd_len, size_t buffer_size, char c)
{
    if (*cmd_len < (int)buffer_size - 1) {
        if (*cursor_pos == *cmd_len) {
            cmd_buffer[*cursor_pos] = c;
            (*cursor_pos)++;
            (*cmd_len)++;
            cmd_buffer[*cmd_len] = '\0';
            char echo[2] = {c, '\0'};
            console_print(echo);
        } else {
            memmove(&cmd_buffer[*cursor_pos + 1], &cmd_buffer[*cursor_pos], *cmd_len - *cursor_pos + 1);
            cmd_buffer[*cursor_pos] = c;
            (*cursor_pos)++;
            (*cmd_len)++;
            console_print(&cmd_buffer[*cursor_pos - 1]);
            int back = *cmd_len - *cursor_pos;
            for (int i = 0; i < back; i++) {
                console_print("\b");
            }
        }
    }
}

static void handle_move_command(const char *args)
{
    while (*args == ' ') args++;

    if (*args == '\0') {
        console_printf("Usage: move <0-110> | move max | move min\r\n");
        console_printf("Current position: %.1f%%\r\n", motor_get_position_percent());
        return;
    }

    float target = 0.0f;
    if (strcasecmp(args, "max") == 0) {
        target = 110.0f;
    } else if (strcasecmp(args, "min") == 0) {
        target = 0.0f;
    } else {
        char *endptr;
        target = strtof(args, &endptr);
        while (*endptr == ' ' || *endptr == '%') endptr++;
        if (endptr == args || (*endptr != '\0' && *endptr != '\r' && *endptr != '\n')) {
            console_printf("Invalid position '%s'. Must be 0.0 to 110.0, 'max', or 'min'\r\n", args);
            return;
        }
    }

    if (target < 0.0f || target > 110.0f) {
        console_printf("Position %.1f%% out of range! Allowed range is 0.0%% to 110.0%%\r\n", target);
        return;
    }

    console_printf("Moving motor to %.1f%%...\r\n", target);
    esp_err_t ret = motor_move_to_percent(target);
    if (ret == ESP_OK) {
        console_printf("Position reached: %.1f%%\r\n", motor_get_position_percent());
    } else if (ret == ESP_ERR_INVALID_STATE) {
        console_printf("Move failed: motor not homed or calibrated! (Type 'f' to home, 'c' to calibrate)\r\n");
    } else {
        console_printf("Move failed: %s\r\n", esp_err_to_name(ret));
    }
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
                handle_move_command("0");
                break;
            case 'm': case 'M':
                handle_move_command("50");
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
                    char pct[16];
                    snprintf(pct, sizeof(pct), "%d", (cmd[0] - '0') * 10);
                    handle_move_command(pct);
                } else {
                    console_printf("Unknown: %s\r\n", cmd);
                }
                break;
        }
        return;
    }
    
    if (strncasecmp(cmd, "move", 4) == 0 && (cmd[4] == '\0' || cmd[4] == ' ')) {
        handle_move_command(cmd + 4);
        return;
    }

    if (strcasecmp(cmd, "max") == 0) {
        handle_move_command("max");
        return;
    }

    if (strcasecmp(cmd, "min") == 0) {
        handle_move_command("min");
        return;
    }

    if (strcmp(cmd, "history") == 0) {
        print_history();
        return;
    }

    if (strcmp(cmd, "drop") == 0) {
        handle_dropoff_command("start");
        return;
    }

    if (strncmp(cmd, "dropoff", 7) == 0 && (cmd[7] == '\0' || cmd[7] == ' ')) {
        handle_dropoff_command(cmd + 7);
        return;
    }

    if (strncasecmp(cmd, "speed", 5) == 0 && (cmd[5] == '\0' || cmd[5] == ' ')) {
        const char *arg = cmd + 5;
        while (*arg == ' ') arg++;
        if (*arg == '\0') {
            console_printf("Current motor speed: %lu RPM\r\n", (unsigned long)motor_get_speed_rpm());
        } else {
            int val = atoi(arg);
            if (val >= 10 && val <= 800) {
                motor_set_speed_rpm((uint32_t)val);
                console_printf("Motor speed set to %d RPM.\r\n", val);
            } else {
                console_printf("Invalid speed (must be 10-800 RPM): %d\r\n", val);
            }
        }
        return;
    }

    if (strncasecmp(cmd, "cycles", 6) == 0 && (cmd[6] == '\0' || cmd[6] == ' ')) {
        const char *arg = cmd + 6;
        while (*arg == ' ') arg++;
        if (*arg == '\0') {
            motor_dropoff_config_t cfg;
            motor_get_dropoff_config(&cfg);
            console_printf("Current dropoff cycles: %d\r\n", cfg.cycles);
        } else {
            char dropoff_arg[32];
            snprintf(dropoff_arg, sizeof(dropoff_arg), "cycles %s", arg);
            handle_dropoff_command(dropoff_arg);
        }
        return;
    }

    /* Multi-character - check if direct number (e.g. 110, 98.5, 101%) */
    char *endptr;
    (void)strtof(cmd, &endptr);
    while (*endptr == ' ' || *endptr == '%') endptr++;
    if (endptr != cmd && (*endptr == '\0' || *endptr == '\r' || *endptr == '\n')) {
        handle_move_command(cmd);
        return;
    }

    console_printf("Unknown: %s. Type 'h' for help.\r\n", cmd);
}

/* ============================================
   CONSOLE TASK
   ============================================ */
typedef enum {
    ESC_STATE_IDLE = 0,
    ESC_STATE_ESC,
    ESC_STATE_BRACKET,
    ESC_STATE_O,
    ESC_STATE_PARAM,
} esc_state_t;

static void console_task(void *pvParameters)
{
    char cmd_buffer[128];
    int cmd_len = 0;
    int cursor_pos = 0;
    esc_state_t esc_state = ESC_STATE_IDLE;
    int esc_param = 0;
    bool last_was_cr = false;
    
    console_printf("> ");
    
    while (1) {
        int c = console_getchar();
        
        if (c >= 0) {
            // Handle ANSI escape sequences (Arrow keys, Home, End, Delete)
            if (esc_state == ESC_STATE_ESC) {
                if (c == '[') {
                    esc_state = ESC_STATE_BRACKET;
                } else if (c == 'O') {
                    esc_state = ESC_STATE_O;
                } else {
                    esc_state = ESC_STATE_IDLE;
                }
                continue;
            } else if (esc_state == ESC_STATE_BRACKET) {
                if (c == 'A') {
                    // UP ARROW: recall previous command
                    history_up(cmd_buffer, &cursor_pos, &cmd_len, sizeof(cmd_buffer));
                    esc_state = ESC_STATE_IDLE;
                } else if (c == 'B') {
                    // DOWN ARROW: recall next command
                    history_down(cmd_buffer, &cursor_pos, &cmd_len, sizeof(cmd_buffer));
                    esc_state = ESC_STATE_IDLE;
                } else if (c == 'C') {
                    // RIGHT ARROW: move cursor right
                    cursor_right(&cursor_pos, cmd_len);
                    esc_state = ESC_STATE_IDLE;
                } else if (c == 'D') {
                    // LEFT ARROW: move cursor left
                    cursor_left(&cursor_pos);
                    esc_state = ESC_STATE_IDLE;
                } else if (c == 'H') {
                    // HOME: move cursor to beginning of line
                    cursor_home(&cursor_pos);
                    esc_state = ESC_STATE_IDLE;
                } else if (c == 'F') {
                    // END: move cursor to end of line
                    cursor_end(&cursor_pos, cmd_len);
                    esc_state = ESC_STATE_IDLE;
                } else if (c >= '0' && c <= '9') {
                    esc_param = c - '0';
                    esc_state = ESC_STATE_PARAM;
                } else {
                    esc_state = ESC_STATE_IDLE;
                }
                continue;
            } else if (esc_state == ESC_STATE_O) {
                // Application keypad / cursor mode
                if (c == 'A') {
                    history_up(cmd_buffer, &cursor_pos, &cmd_len, sizeof(cmd_buffer));
                } else if (c == 'B') {
                    history_down(cmd_buffer, &cursor_pos, &cmd_len, sizeof(cmd_buffer));
                } else if (c == 'C') {
                    cursor_right(&cursor_pos, cmd_len);
                } else if (c == 'D') {
                    cursor_left(&cursor_pos);
                } else if (c == 'H') {
                    cursor_home(&cursor_pos);
                } else if (c == 'F') {
                    cursor_end(&cursor_pos, cmd_len);
                }
                esc_state = ESC_STATE_IDLE;
                continue;
            } else if (esc_state == ESC_STATE_PARAM) {
                if (c == '~') {
                    if (esc_param == 3) {
                        // DELETE key
                        handle_delete(cmd_buffer, &cursor_pos, &cmd_len);
                    } else if (esc_param == 1 || esc_param == 7) {
                        // HOME key
                        cursor_home(&cursor_pos);
                    } else if (esc_param == 4 || esc_param == 8) {
                        // END key
                        cursor_end(&cursor_pos, cmd_len);
                    }
                    esc_state = ESC_STATE_IDLE;
                } else if (c >= '0' && c <= '9') {
                    esc_param = esc_param * 10 + (c - '0');
                } else {
                    esc_state = ESC_STATE_IDLE;
                }
                continue;
            }

            // Normal state: check for ESC
            if (c == 0x1B) {
                esc_state = ESC_STATE_ESC;
                continue;
            }

            if (c == '\r' || c == '\n') {
                if (c == '\n' && last_was_cr) {
                    last_was_cr = false;
                    continue; // Skip LF following CR
                }
                last_was_cr = (c == '\r');

                console_printf("\r\n");
                cmd_buffer[cmd_len] = '\0';
                if (cmd_len > 0) {
                    history_add(cmd_buffer);
                    process_command(cmd_buffer);
                }
                cmd_len = 0;
                cursor_pos = 0;
                s_temp_buffer[0] = '\0';
                s_history_pos = s_history_count;
                console_printf("> ");
            } else {
                last_was_cr = false;

                if (c == 127 || c == 8) {
                    // Backspace
                    handle_backspace(cmd_buffer, &cursor_pos, &cmd_len);
                } else if (c >= 32 && c < 127) {
                    // Printable character insertion
                    handle_insert_char(cmd_buffer, &cursor_pos, &cmd_len, sizeof(cmd_buffer), (char)c);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void console_start_task(void)
{
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
}