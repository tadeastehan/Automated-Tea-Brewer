/**
 * @file main.c
 * @brief TMC2130 Stepper Motor Control Example for ESP-IDF
 * 
 * This example demonstrates basic motor control using the TMC2130 driver
 * including StealthChop, SpreadCycle, and StallGuard features.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "tmc2130.h"

static const char *TAG = "TMC2130_MAIN";

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
#define MOTOR_RMS_CURRENT_MA    800     // RMS current in mA
#define MOTOR_HOLD_MULTIPLIER   0.5f    // Hold current as fraction of run current
#define MOTOR_MICROSTEPS        16      // Microsteps per full step
#define MOTOR_R_SENSE           0.11f   // Sense resistor value in ohms

/* Stepping configuration */
#define STEPS_PER_REV           200     // Full steps per revolution (1.8° motor)
#define DEFAULT_SPEED_RPM       60      // Default speed in RPM

/* Global handle */
static tmc2130_handle_t tmc2130;
static gptimer_handle_t step_timer = NULL;
static volatile bool motor_running = false;
static volatile int32_t steps_remaining = 0;
static volatile bool step_state = false;

/**
 * @brief Timer callback for generating step pulses
 */
static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    if (steps_remaining > 0 || steps_remaining == -1) {  // -1 = continuous
        step_state = !step_state;
        gpio_set_level(STEP_PIN, step_state);
        
        if (step_state && steps_remaining > 0) {
            steps_remaining--;
        }
    } else {
        motor_running = false;
        gptimer_stop(timer);
    }
    
    return false;  // No need to yield
}

/**
 * @brief Initialize the step timer for precise stepping
 */
static esp_err_t init_step_timer(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1MHz, 1 tick = 1us
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
    
    // Calculate step interval in microseconds
    // steps_per_second = (rpm * STEPS_PER_REV * MOTOR_MICROSTEPS) / 60
    // interval_us = 1000000 / steps_per_second / 2 (divide by 2 for toggle)
    uint32_t steps_per_second = (rpm * STEPS_PER_REV * MOTOR_MICROSTEPS) / 60;
    uint32_t interval_us = 500000 / steps_per_second;  // Half period for toggle
    
    if (interval_us < 10) {
        interval_us = 10;  // Minimum 10us interval
    }
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = interval_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    
    ESP_ERROR_CHECK(gptimer_set_alarm_action(step_timer, &alarm_config));
    
    ESP_LOGI(TAG, "Speed set to %lu RPM (interval: %lu us)", rpm, interval_us);
    return ESP_OK;
}

/**
 * @brief Move motor a specific number of steps
 */
static void move_steps(int32_t steps)
{
    if (steps == 0) return;
    
    // Set direction
    bool dir = (steps > 0);
    tmc2130_set_direction(&tmc2130, dir);
    gpio_set_level(DIR_PIN, dir);
    vTaskDelay(pdMS_TO_TICKS(1));  // Direction setup time
    
    steps_remaining = (steps > 0) ? steps : -steps;
    motor_running = true;
    
    gptimer_set_raw_count(step_timer, 0);
    gptimer_start(step_timer);
    
    ESP_LOGI(TAG, "Moving %ld steps %s", steps, dir ? "CW" : "CCW");
}

/**
 * @brief Move motor continuously
 */
static void move_continuous(bool direction)
{
    tmc2130_set_direction(&tmc2130, direction);
    gpio_set_level(DIR_PIN, direction);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    steps_remaining = -1;  // Continuous mode
    motor_running = true;
    
    gptimer_set_raw_count(step_timer, 0);
    gptimer_start(step_timer);
    
    ESP_LOGI(TAG, "Continuous rotation %s", direction ? "CW" : "CCW");
}

/**
 * @brief Stop motor movement
 */
static void stop_motor(void)
{
    steps_remaining = 0;
    motor_running = false;
    gptimer_stop(step_timer);
    gpio_set_level(STEP_PIN, 0);
    
    ESP_LOGI(TAG, "Motor stopped");
}

/**
 * @brief Wait for movement to complete
 */
static void wait_for_movement(void)
{
    while (motor_running) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Print driver status
 */
static void print_driver_status(void)
{
    uint32_t status;
    if (tmc2130_get_drv_status(&tmc2130, &status) == ESP_OK) {
        ESP_LOGI(TAG, "DRV_STATUS: 0x%08lX", status);
        ESP_LOGI(TAG, "  SG_RESULT: %u", (unsigned int)(status & 0x3FF));
        ESP_LOGI(TAG, "  fsactive: %s", (status & (1 << 15)) ? "yes" : "no");
        ESP_LOGI(TAG, "  CS_ACTUAL: %u", (unsigned int)((status >> 16) & 0x1F));
        ESP_LOGI(TAG, "  stallGuard: %s", (status & (1 << 24)) ? "STALL" : "ok");
        ESP_LOGI(TAG, "  ot: %s", (status & (1 << 25)) ? "OVERTEMP" : "ok");
        ESP_LOGI(TAG, "  otpw: %s", (status & (1 << 26)) ? "warning" : "ok");
        ESP_LOGI(TAG, "  s2ga: %s", (status & (1 << 27)) ? "short" : "ok");
        ESP_LOGI(TAG, "  s2gb: %s", (status & (1 << 28)) ? "short" : "ok");
        ESP_LOGI(TAG, "  ola: %s", (status & (1 << 29)) ? "open" : "ok");
        ESP_LOGI(TAG, "  olb: %s", (status & (1 << 30)) ? "open" : "ok");
        ESP_LOGI(TAG, "  stst: %s", (status & (1UL << 31)) ? "standstill" : "moving");
    }
}

/**
 * @brief Configure TMC2130 for StealthChop mode (quiet operation)
 */
static void configure_stealthchop(void)
{
    ESP_LOGI(TAG, "Configuring StealthChop mode...");
    
    // Enable StealthChop
    tmc2130_set_en_pwm_mode(&tmc2130, true);
    
    // Configure PWMCONF for StealthChop
    tmc2130_set_pwm_autoscale(&tmc2130, true);
    tmc2130_set_pwm_freq(&tmc2130, 1);      // PWM frequency: 2/683 fclk
    tmc2130_set_pwm_ampl(&tmc2130, 255);    // PWM amplitude
    tmc2130_set_pwm_grad(&tmc2130, 5);      // PWM gradient
    
    // Set threshold for switching to SpreadCycle at high speeds
    tmc2130_set_tpwmthrs(&tmc2130, 500);    // Switch threshold
    
    ESP_LOGI(TAG, "StealthChop configured");
}

/**
 * @brief Configure TMC2130 for SpreadCycle mode (high torque)
 */
static void configure_spreadcycle(void)
{
    ESP_LOGI(TAG, "Configuring SpreadCycle mode...");
    
    // Disable StealthChop (use SpreadCycle)
    tmc2130_set_en_pwm_mode(&tmc2130, false);
    
    // Configure CHOPCONF for SpreadCycle
    tmc2130_set_toff(&tmc2130, 4);          // Off time: 4
    tmc2130_set_hstrt(&tmc2130, 4);         // Hysteresis start: 4
    tmc2130_set_hend(&tmc2130, 1);          // Hysteresis end: 1
    tmc2130_set_tbl(&tmc2130, 2);           // Blanking time: 2
    tmc2130_set_chm(&tmc2130, false);       // SpreadCycle mode
    
    ESP_LOGI(TAG, "SpreadCycle configured");
}

/**
 * @brief Configure StallGuard for sensorless homing
 */
static void configure_stallguard(int8_t threshold)
{
    ESP_LOGI(TAG, "Configuring StallGuard with threshold: %d", threshold);
    
    // Configure CoolStep thresholds
    tmc2130_set_tcoolthrs(&tmc2130, 0xFFFFF);  // Enable StallGuard at all speeds
    
    // Set StallGuard threshold
    tmc2130_set_sgt(&tmc2130, threshold);
    
    // Enable StallGuard output on DIAG1
    tmc2130_set_diag1_stall(&tmc2130, true);
    tmc2130_set_diag1_pushpull(&tmc2130, true);
    
    // Optional: Enable StallGuard filter
    tmc2130_set_sfilt(&tmc2130, true);
    
    ESP_LOGI(TAG, "StallGuard configured");
}

/**
 * @brief Demo: Basic movement
 */
static void demo_basic_movement(void)
{
    ESP_LOGI(TAG, "=== Demo: Basic Movement ===");
    
    set_speed_rpm(60);
    
    // Move one full revolution clockwise
    ESP_LOGI(TAG, "Moving 1 revolution CW...");
    move_steps(STEPS_PER_REV * MOTOR_MICROSTEPS);
    wait_for_movement();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Move one full revolution counter-clockwise
    ESP_LOGI(TAG, "Moving 1 revolution CCW...");
    move_steps(-(STEPS_PER_REV * MOTOR_MICROSTEPS));
    wait_for_movement();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    print_driver_status();
}

/**
 * @brief Demo: Speed ramping
 */
static void demo_speed_ramp(void)
{
    ESP_LOGI(TAG, "=== Demo: Speed Ramping ===");
    
    // Start slow and increase speed
    for (int rpm = 10; rpm <= 200; rpm += 10) {
        set_speed_rpm(rpm);
        move_continuous(true);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Decrease speed
    for (int rpm = 200; rpm >= 10; rpm -= 10) {
        set_speed_rpm(rpm);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(500));
}

/**
 * @brief Demo: Compare StealthChop vs SpreadCycle
 */
static void demo_chopper_modes(void)
{
    ESP_LOGI(TAG, "=== Demo: Chopper Modes ===");
    
    set_speed_rpm(60);
    
    // StealthChop mode (quiet)
    ESP_LOGI(TAG, "StealthChop mode (listen for quiet operation)...");
    configure_stealthchop();
    move_steps(STEPS_PER_REV * MOTOR_MICROSTEPS * 2);
    wait_for_movement();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // SpreadCycle mode (more torque, more noise)
    ESP_LOGI(TAG, "SpreadCycle mode (more torque)...");
    configure_spreadcycle();
    move_steps(STEPS_PER_REV * MOTOR_MICROSTEPS * 2);
    wait_for_movement();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Return to StealthChop
    configure_stealthchop();
}

/**
 * @brief Demo: StallGuard monitoring
 */
static void demo_stallguard(void)
{
    ESP_LOGI(TAG, "=== Demo: StallGuard Monitoring ===");
    
    // Must use SpreadCycle for StallGuard
    configure_spreadcycle();
    configure_stallguard(10);  // Threshold: 10
    
    set_speed_rpm(30);
    move_continuous(true);
    
    // Monitor StallGuard value
    for (int i = 0; i < 50; i++) {
        uint16_t sg_result = tmc2130_get_sg_result(&tmc2130);
        bool stall = tmc2130_is_stallguard(&tmc2130);
        
        ESP_LOGI(TAG, "SG_RESULT: %u %s", sg_result, stall ? "STALL!" : "");
        
        if (stall) {
            ESP_LOGW(TAG, "Stall detected! Stopping motor.");
            stop_motor();
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    stop_motor();
    
    // Return to StealthChop
    configure_stealthchop();
}

/**
 * @brief Initialize TMC2130 driver
 */
static esp_err_t init_tmc2130(void)
{
    ESP_LOGI(TAG, "Initializing TMC2130...");
    
    tmc2130_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_mosi = MOSI_PIN,
        .pin_miso = MISO_PIN,
        .pin_sclk = SCLK_PIN,
        .pin_cs = CS_PIN,
        .pin_en = EN_PIN,
        .pin_step = STEP_PIN,
        .pin_dir = DIR_PIN,
        .spi_clock_speed_hz = 1000000,  // 1 MHz
        .r_sense = MOTOR_R_SENSE,
    };
    
    esp_err_t ret = tmc2130_init(&tmc2130, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TMC2130: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Test connection
    if (!tmc2130_test_connection(&tmc2130)) {
        ESP_LOGW(TAG, "TMC2130 connection test failed - check wiring!");
    }
    
    // Basic configuration
    tmc2130_set_i_scale_analog(&tmc2130, false);    // Use internal reference
    tmc2130_set_internal_rsense(&tmc2130, false);   // Use external sense resistors
    
    // Set current
    tmc2130_set_rms_current(&tmc2130, MOTOR_RMS_CURRENT_MA, MOTOR_HOLD_MULTIPLIER);
    
    // Set microsteps
    tmc2130_set_microsteps(&tmc2130, MOTOR_MICROSTEPS);
    tmc2130_set_intpol(&tmc2130, true);  // Interpolate to 256 microsteps
    
    // Configure chopper
    tmc2130_set_toff(&tmc2130, 4);
    tmc2130_set_tbl(&tmc2130, 2);
    tmc2130_set_hstrt(&tmc2130, 4);
    tmc2130_set_hend(&tmc2130, 1);
    
    // Enable StealthChop by default
    configure_stealthchop();
    
    // Set power down delay
    tmc2130_set_tpowerdown(&tmc2130, 10);
    
    // Enable driver
    tmc2130_enable(&tmc2130);
    
    ESP_LOGI(TAG, "TMC2130 initialization complete");
    
    // Print initial status
    print_driver_status();
    
    return ESP_OK;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "TMC2130 Stepper Motor Control Example");
    ESP_LOGI(TAG, "======================================");
    
    // Initialize step timer
    ESP_ERROR_CHECK(init_step_timer());
    
    // Initialize TMC2130
    if (init_tmc2130() != ESP_OK) {
        ESP_LOGE(TAG, "TMC2130 initialization failed!");
        return;
    }
    
    // Wait for driver to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Run demos
    while (1) {
        ESP_LOGI(TAG, "\n\n--- Starting Demo Sequence ---\n");
        
        // Demo 1: Basic movement
        demo_basic_movement();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Demo 2: Speed ramping
        demo_speed_ramp();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Demo 3: Chopper mode comparison
        demo_chopper_modes();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Demo 4: StallGuard monitoring
        demo_stallguard();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Print final status
        print_driver_status();
        
        ESP_LOGI(TAG, "\n--- Demo sequence complete. Restarting in 5 seconds... ---\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}