/**
 * @file main.c
 * @brief TMC2130 Sensorless Homing and Position Control for ESP-IDF
 * 
 * Serial Commands:
 *   c        - Start calibration
 *   0-100    - Move to percentage position
 *   i        - Increase StallGuard threshold
 *   d        - Decrease StallGuard threshold
 *   s        - Show current status
 *   h        - Show help
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "tmc2130.h"

static const char *TAG = "TMC2130_CTRL";

/* User-provided pins */
#define EN_PIN    2
#define DIR_PIN   0
#define STEP_PIN  1
#define CS_PIN    21

/* SPI pins */
#define SCLK_PIN  19
#define MOSI_PIN  18
#define MISO_PIN  20

/* Motor configuration */
#define MOTOR_RMS_CURRENT_MA    800
#define MOTOR_HOLD_MULTIPLIER   0.5f
#define MOTOR_MICROSTEPS        16
#define MOTOR_R_SENSE           0.11f

/* Stepping configuration */
#define STEPS_PER_REV           200

/* Homing configuration */
#define HOMING_SPEED_RPM        30
#define NORMAL_SPEED_RPM        120
#define STALL_DEBOUNCE_MS       50
#define BACKOFF_STEPS           100

/* Global handle */
static tmc2130_handle_t tmc2130;
static gptimer_handle_t step_timer = NULL;

/* Motor state */
static volatile bool motor_running = false;
static volatile int32_t steps_remaining = 0;
static volatile bool step_state = false;
static volatile int32_t current_position = 0;
static volatile bool direction_forward = true;

/* Calibration data */
static bool is_calibrated = false;
static int32_t min_position = 0;
static int32_t max_position = 0;
static int32_t total_travel_steps = 0;

/* StallGuard threshold - adjustable via serial */
static int8_t stallguard_threshold = -6;

/**
 * @brief Initialize USB Serial JTAG for console (ESP32-C6)
 */
static void init_console(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));
    esp_vfs_usb_serial_jtag_use_driver();
    
    printf("\r\n");
}

/**
 * @brief Timer callback for generating step pulses
 */
static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    if (steps_remaining > 0 || steps_remaining == -1) {
        step_state = !step_state;
        gpio_set_level(STEP_PIN, step_state);
        
        if (step_state) {
            if (steps_remaining > 0) {
                steps_remaining--;
            }
            if (direction_forward) {
                current_position++;
            } else {
                current_position--;
            }
        }
    } else {
        motor_running = false;
        gptimer_stop(timer);
    }
    
    return false;
}

/**
 * @brief Initialize the step timer
 */
static esp_err_t init_step_timer(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &step_timer));
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(step_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(step_timer));
    
    return ESP_OK;
}

/**
 * @brief Set motor speed in RPM
 */
static esp_err_t set_speed_rpm(uint32_t rpm)
{
    if (rpm == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t steps_per_second = (rpm * STEPS_PER_REV * MOTOR_MICROSTEPS) / 60;
    uint32_t interval_us = 500000 / steps_per_second;
    
    if (interval_us < 10) {
        interval_us = 10;
    }
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = interval_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    
    ESP_ERROR_CHECK(gptimer_set_alarm_action(step_timer, &alarm_config));
    
    return ESP_OK;
}

/**
 * @brief Set direction
 */
static void set_direction(bool forward)
{
    direction_forward = forward;
    gpio_set_level(DIR_PIN, forward ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * @brief Start continuous movement
 */
static void start_continuous_movement(bool forward)
{
    set_direction(forward);
    steps_remaining = -1;
    motor_running = true;
    
    gptimer_set_raw_count(step_timer, 0);
    gptimer_start(step_timer);
}

/**
 * @brief Move specific number of steps (non-blocking check with proper yields)
 */
static void move_steps_blocking(int32_t steps)
{
    if (steps == 0) return;
    
    bool forward = (steps > 0);
    set_direction(forward);
    
    steps_remaining = (steps > 0) ? steps : -steps;
    motor_running = true;
    
    gptimer_set_raw_count(step_timer, 0);
    gptimer_start(step_timer);
    
    // Wait for completion with proper delays to prevent watchdog timeout
    int32_t last_reported = steps_remaining;
    while (motor_running) {
        // Use longer delay to give IDLE task time to run
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Optional: print progress for long moves
        if (steps_remaining < last_reported - 1000) {
            printf("  Progress: %ld steps remaining...\r\n", (long)steps_remaining);
            last_reported = steps_remaining;
        }
    }
}

/**
 * @brief Stop motor
 */
static void stop_motor(void)
{
    steps_remaining = 0;
    motor_running = false;
    gptimer_stop(step_timer);
    gpio_set_level(STEP_PIN, 0);
}

/**
 * @brief Check for stall condition with debouncing
 */
static bool check_stall(void)
{
    static int64_t last_stall_time = 0;
    
    uint16_t sg_result = tmc2130_get_sg_result(&tmc2130);
    bool stall_flag = tmc2130_is_stallguard(&tmc2130);
    
    static int log_counter = 0;
    if (++log_counter >= 20) {
        printf("SG: %u %s\r\n", sg_result, stall_flag ? "STALL" : "");
        log_counter = 0;
    }
    
    if (stall_flag || sg_result == 0) {
        int64_t now = esp_timer_get_time() / 1000;
        if (last_stall_time == 0) {
            last_stall_time = now;
        } else if ((now - last_stall_time) >= STALL_DEBOUNCE_MS) {
            last_stall_time = 0;
            return true;
        }
    } else {
        last_stall_time = 0;
    }
    
    return false;
}

/**
 * @brief Configure driver for StallGuard homing
 */
static void configure_for_homing(void)
{
    printf("Configuring for homing (SGT: %d)...\r\n", stallguard_threshold);
    
    tmc2130_set_en_pwm_mode(&tmc2130, false);
    tmc2130_set_toff(&tmc2130, 4);
    tmc2130_set_hstrt(&tmc2130, 4);
    tmc2130_set_hend(&tmc2130, 1);
    tmc2130_set_tbl(&tmc2130, 2);
    tmc2130_set_chm(&tmc2130, false);
    tmc2130_set_tcoolthrs(&tmc2130, 0xFFFFF);
    tmc2130_set_sgt(&tmc2130, stallguard_threshold);
    tmc2130_set_sfilt(&tmc2130, true);
    tmc2130_set_diag1_stall(&tmc2130, true);
    tmc2130_set_diag1_pushpull(&tmc2130, true);
}

/**
 * @brief Configure driver for normal operation
 */
static void configure_for_normal_operation(void)
{
    printf("Configuring for normal operation...\r\n");
    
    tmc2130_set_en_pwm_mode(&tmc2130, true);
    tmc2130_set_pwm_autoscale(&tmc2130, true);
    tmc2130_set_pwm_freq(&tmc2130, 1);
    tmc2130_set_pwm_ampl(&tmc2130, 255);
    tmc2130_set_pwm_grad(&tmc2130, 5);
    tmc2130_set_diag1_stall(&tmc2130, false);
}

/**
 * @brief Find endpoint by moving until stall
 */
static int32_t find_endpoint(bool forward)
{
    printf("Finding %s endpoint...\r\n", forward ? "forward" : "reverse");
    
    int32_t start_position = current_position;
    
    set_speed_rpm(HOMING_SPEED_RPM);
    start_continuous_movement(forward);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while (motor_running) {
        if (check_stall()) {
            stop_motor();
            printf("Stall detected at position: %ld\r\n", (long)current_position);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Increased delay
    }
    
    int32_t steps_moved = current_position - start_position;
    if (steps_moved < 0) steps_moved = -steps_moved;
    
    return steps_moved;
}

/**
 * @brief Perform sensorless homing calibration
 */
static bool perform_calibration(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("   SENSORLESS HOMING CALIBRATION\r\n");
    printf("   StallGuard Threshold: %d\r\n", stallguard_threshold);
    printf("========================================\r\n");
    printf("\r\n");
    
    is_calibrated = false;
    
    configure_for_homing();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    printf("Step 1: Finding first endpoint...\r\n");
    find_endpoint(false);
    
    printf("Backing off...\r\n");
    move_steps_blocking(BACKOFF_STEPS);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    find_endpoint(false);
    
    current_position = 0;
    min_position = 0;
    printf("Endpoint 1 set as position 0\r\n");
    
    move_steps_blocking(BACKOFF_STEPS);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    printf("Step 2: Finding second endpoint...\r\n");
    find_endpoint(true);
    
    max_position = current_position;
    
    printf("Backing off...\r\n");
    move_steps_blocking(-BACKOFF_STEPS);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    find_endpoint(true);
    max_position = current_position;
    
    total_travel_steps = max_position - min_position;
    
    printf("\r\n");
    printf("========================================\r\n");
    printf("   CALIBRATION COMPLETE\r\n");
    printf("========================================\r\n");
    printf("Min position:  %ld steps\r\n", (long)min_position);
    printf("Max position:  %ld steps\r\n", (long)max_position);
    printf("Total travel:  %ld steps\r\n", (long)total_travel_steps);
    printf("Travel (mm):   %.2f mm @ 80 steps/mm\r\n", (float)total_travel_steps / 80.0f);
    printf("========================================\r\n");
    printf("\r\n");
    
    move_steps_blocking(-BACKOFF_STEPS);
    
    configure_for_normal_operation();
    set_speed_rpm(NORMAL_SPEED_RPM);
    
    is_calibrated = true;
    return true;
}

/**
 * @brief Move to absolute position
 */
static bool move_to_position(int32_t target_position)
{
    if (!is_calibrated) {
        printf("Error: Not calibrated! Press 'c' to calibrate.\r\n");
        return false;
    }
    
    int32_t safe_min = min_position + BACKOFF_STEPS;
    int32_t safe_max = max_position - BACKOFF_STEPS;
    
    if (target_position < safe_min) {
        printf("Clamping to safe min: %ld\r\n", (long)safe_min);
        target_position = safe_min;
    }
    if (target_position > safe_max) {
        printf("Clamping to safe max: %ld\r\n", (long)safe_max);
        target_position = safe_max;
    }
    
    int32_t steps_to_move = target_position - current_position;
    
    printf("Moving: %ld -> %ld (%ld steps)\r\n", 
           (long)current_position, (long)target_position, (long)steps_to_move);
    
    if (steps_to_move == 0) {
        printf("Already at target.\r\n");
        return true;
    }
    
    move_steps_blocking(steps_to_move);
    
    printf("Done. Position: %ld\r\n", (long)current_position);
    return true;
}

/**
 * @brief Move to percentage of travel range
 */
static bool move_to_percent(float percent)
{
    if (!is_calibrated) {
        printf("Error: Not calibrated! Press 'c' to calibrate.\r\n");
        return false;
    }
    
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    int32_t usable_min = min_position + BACKOFF_STEPS;
    int32_t usable_max = max_position - BACKOFF_STEPS;
    int32_t usable_range = usable_max - usable_min;
    
    int32_t target = usable_min + (int32_t)((float)usable_range * percent / 100.0f);
    
    printf("Moving to %.1f%%...\r\n", percent);
    return move_to_position(target);
}

/**
 * @brief Get current position as percentage
 */
static float get_position_percent(void)
{
    if (!is_calibrated || total_travel_steps == 0) {
        return 0.0f;
    }
    
    int32_t usable_min = min_position + BACKOFF_STEPS;
    int32_t usable_max = max_position - BACKOFF_STEPS;
    int32_t usable_range = usable_max - usable_min;
    
    if (usable_range <= 0) return 0.0f;
    
    float percent = ((float)(current_position - usable_min) / (float)usable_range) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    return percent;
}

/**
 * @brief Print current status
 */
static void print_status(void)
{
    printf("\r\n");
    printf("=== STATUS ===\r\n");
    printf("Calibrated:     %s\r\n", is_calibrated ? "YES" : "NO");
    printf("SGT Threshold:  %d\r\n", stallguard_threshold);
    printf("Position:       %ld steps\r\n", (long)current_position);
    
    if (is_calibrated) {
        printf("Position %%:     %.1f%%\r\n", get_position_percent());
        printf("Range:          %ld to %ld steps\r\n", (long)min_position, (long)max_position);
        printf("Total travel:   %ld steps\r\n", (long)total_travel_steps);
    }
    
    uint32_t status;
    if (tmc2130_get_drv_status(&tmc2130, &status) == ESP_OK) {
        printf("Motor:          %s\r\n", (status & (1UL << 31)) ? "STANDSTILL" : "MOVING");
        printf("SG Result:      %lu\r\n", (unsigned long)(status & 0x3FF));
    }
    printf("==============\r\n");
    printf("\r\n");
}

/**
 * @brief Print help message
 */
static void print_help(void)
{
    printf("\r\n");
    printf("=== TMC2130 CONTROL COMMANDS ===\r\n");
    printf("  c       - Start calibration\r\n");
    printf("  0-100   - Move to percentage (e.g., '50' for 50%%)\r\n");
    printf("  i       - Increase StallGuard threshold (+1)\r\n");
    printf("  d       - Decrease StallGuard threshold (-1)\r\n");
    printf("  s       - Show current status\r\n");
    printf("  h       - Show this help\r\n");
    printf("  r       - Re-home (go to 0%%)\r\n");
    printf("  m       - Go to middle (50%%)\r\n");
    printf("  e       - Enable motor driver\r\n");
    printf("  x       - Emergency stop / disable motor\r\n");
    printf("  t       - Test StallGuard (show SG values)\r\n");
    printf("================================\r\n");
    printf("\r\n");
}

/**
 * @brief Increase StallGuard threshold
 */
static void increase_sgt(void)
{
    if (stallguard_threshold < 63) {
        stallguard_threshold++;
        tmc2130_set_sgt(&tmc2130, stallguard_threshold);
        printf("StallGuard threshold: %d\r\n", stallguard_threshold);
    } else {
        printf("StallGuard threshold at maximum (63)\r\n");
    }
}

/**
 * @brief Decrease StallGuard threshold
 */
static void decrease_sgt(void)
{
    if (stallguard_threshold > -64) {
        stallguard_threshold--;
        tmc2130_set_sgt(&tmc2130, stallguard_threshold);
        printf("StallGuard threshold: %d\r\n", stallguard_threshold);
    } else {
        printf("StallGuard threshold at minimum (-64)\r\n");
    }
}

/**
 * @brief Test StallGuard - show live SG values
 */
static void test_stallguard(void)
{
    printf("\r\n");
    printf("=== STALLGUARD TEST ===\r\n");
    printf("Showing SG values for 5 seconds...\r\n");
    printf("Move the motor manually to see values change.\r\n");
    printf("\r\n");
    
    configure_for_homing();
    
    for (int i = 0; i < 50; i++) {
        uint16_t sg = tmc2130_get_sg_result(&tmc2130);
        bool stall = tmc2130_is_stallguard(&tmc2130);
        
        printf("SG: %4u  %s\r\n", sg, stall ? "<-- STALL" : "");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    configure_for_normal_operation();
    printf("Test complete.\r\n\r\n");
}

/**
 * @brief Process received command
 */
static void process_command(const char *cmd)
{
    if (strlen(cmd) == 0) {
        return;
    }
    
    // Single character commands
    if (strlen(cmd) == 1) {
        switch (cmd[0]) {
            case 'c':
            case 'C':
                perform_calibration();
                break;
                
            case 'i':
            case 'I':
                increase_sgt();
                break;
                
            case 'd':
            case 'D':
                decrease_sgt();
                break;
                
            case 's':
            case 'S':
                print_status();
                break;
                
            case 'h':
            case 'H':
            case '?':
                print_help();
                break;
                
            case 'r':
            case 'R':
                move_to_percent(0.0f);
                break;
                
            case 'm':
            case 'M':
                move_to_percent(50.0f);
                break;
                
            case 'e':
            case 'E':
                tmc2130_enable(&tmc2130);
                printf("Motor enabled.\r\n");
                break;
                
            case 'x':
            case 'X':
                stop_motor();
                tmc2130_disable(&tmc2130);
                printf("EMERGENCY STOP - Motor disabled.\r\n");
                break;
                
            case 't':
            case 'T':
                test_stallguard();
                break;
                
            default:
                if (cmd[0] >= '0' && cmd[0] <= '9') {
                    float percent = (float)(cmd[0] - '0') * 10.0f;
                    move_to_percent(percent);
                } else {
                    printf("Unknown command: '%s'. Type 'h' for help.\r\n", cmd);
                }
                break;
        }
        return;
    }
    
    // Multi-character commands - try to parse as number
    char *endptr;
    float value = strtof(cmd, &endptr);
    
    if (endptr != cmd && (*endptr == '\0' || *endptr == '\r' || *endptr == '\n')) {
        if (value >= 0.0f && value <= 100.0f) {
            move_to_percent(value);
        } else {
            printf("Position must be 0-100. Got: %.1f\r\n", value);
        }
    } else {
        printf("Unknown command: '%s'. Type 'h' for help.\r\n", cmd);
    }
}

/**
 * @brief Initialize TMC2130 driver
 */
static esp_err_t init_tmc2130(void)
{
    printf("Initializing TMC2130...\r\n");
    
    tmc2130_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_mosi = MOSI_PIN,
        .pin_miso = MISO_PIN,
        .pin_sclk = SCLK_PIN,
        .pin_cs = CS_PIN,
        .pin_en = EN_PIN,
        .pin_step = STEP_PIN,
        .pin_dir = DIR_PIN,
        .spi_clock_speed_hz = 1000000,
        .r_sense = MOTOR_R_SENSE,
    };
    
    esp_err_t ret = tmc2130_init(&tmc2130, &config);
    if (ret != ESP_OK) {
        printf("Failed to initialize TMC2130: %s\r\n", esp_err_to_name(ret));
        return ret;
    }
    
    if (!tmc2130_test_connection(&tmc2130)) {
        printf("WARNING: TMC2130 connection test failed!\r\n");
    } else {
        printf("TMC2130 connected.\r\n");
    }
    
    tmc2130_set_i_scale_analog(&tmc2130, false);
    tmc2130_set_internal_rsense(&tmc2130, false);
    tmc2130_set_rms_current(&tmc2130, MOTOR_RMS_CURRENT_MA, MOTOR_HOLD_MULTIPLIER);
    tmc2130_set_microsteps(&tmc2130, MOTOR_MICROSTEPS);
    tmc2130_set_intpol(&tmc2130, true);
    tmc2130_set_toff(&tmc2130, 4);
    tmc2130_set_tbl(&tmc2130, 2);
    tmc2130_set_hstrt(&tmc2130, 4);
    tmc2130_set_hend(&tmc2130, 1);
    tmc2130_set_tpowerdown(&tmc2130, 10);
    tmc2130_set_sgt(&tmc2130, stallguard_threshold);
    
    tmc2130_enable(&tmc2130);
    
    printf("TMC2130 initialized.\r\n");
    return ESP_OK;
}

/**
 * @brief Motor control task - handles movement without blocking serial
 */
typedef struct {
    bool pending;
    float target_percent;
} move_command_t;

static volatile move_command_t pending_move = {false, 0};

static void motor_task(void *pvParameters)
{
    while (1) {
        if (pending_move.pending) {
            pending_move.pending = false;
            move_to_percent(pending_move.target_percent);
            printf("> ");
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Serial input task
 */
static void serial_task(void *pvParameters)
{
    char cmd_buffer[64];
    int cmd_index = 0;
    
    printf("> ");
    fflush(stdout);
    
    while (1) {
        int c = getchar();
        
        if (c != EOF) {
            if (c == '\r' || c == '\n') {
                printf("\r\n");
                cmd_buffer[cmd_index] = '\0';
                
                if (cmd_index > 0) {
                    process_command(cmd_buffer);
                }
                
                cmd_index = 0;
                printf("> ");
                fflush(stdout);
            } else if (c == 127 || c == 8) {
                if (cmd_index > 0) {
                    cmd_index--;
                    printf("\b \b");
                    fflush(stdout);
                }
            } else if (c >= 32 && c < 127 && cmd_index < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_index++] = (char)c;
                putchar(c);
                fflush(stdout);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    // Initialize console FIRST
    init_console();
    
    printf("\r\n");
    printf("========================================\r\n");
    printf("  TMC2130 Sensorless Homing Controller\r\n");
    printf("  For ESP32-C6\r\n");
    printf("========================================\r\n");
    printf("\r\n");
    
    // Initialize step timer
    ESP_ERROR_CHECK(init_step_timer());
    
    // Initialize TMC2130
    if (init_tmc2130() != ESP_OK) {
        printf("TMC2130 initialization failed!\r\n");
        return;
    }
    
    // Configure for normal operation initially
    configure_for_normal_operation();
    set_speed_rpm(NORMAL_SPEED_RPM);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Print help
    print_help();
    
    printf("Ready. Type 'c' to calibrate or 'h' for help.\r\n");
    
    // Create serial input task with higher priority
    xTaskCreate(serial_task, "serial_task", 4096, NULL, 10, NULL);
    
    // Main loop - just keep alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}