/**
 * @file motor_control.c
 * @brief High-level motor control implementation
 */

#include "motor_control.h"
#include "tmc2130.h"
#include "../main_pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

/* ============================================
   CONFIGURATION
   ============================================ */
#define MOTOR_RMS_CURRENT_MA    800
#define MOTOR_HOLD_MULTIPLIER   0.5f
#define MOTOR_MICROSTEPS        16
#define MOTOR_R_SENSE           0.11f
#define STEPS_PER_REV           200

#define HOMING_SPEED_RPM        30
#define NORMAL_SPEED_RPM        120
#define STALL_DEBOUNCE_MS       50
#define DEFAULT_BACKOFF_STEPS   100

/* NVS Keys */
#define NVS_NAMESPACE           "motor"
#define NVS_KEY_CALIBRATED      "cal"
#define NVS_KEY_TOTAL_STEPS     "steps"
#define NVS_KEY_SGT             "sgt"
#define NVS_KEY_BACKOFF         "backoff"

/* ============================================
   PRIVATE DATA
   ============================================ */
static tmc2130_handle_t tmc2130;
static gptimer_handle_t step_timer = NULL;

/* Motor state */
static volatile bool motor_running = false;
static volatile int32_t steps_remaining = 0;
static volatile bool step_state = false;
static volatile int32_t current_position = 0;
static volatile bool direction_forward = true;
static motor_state_t current_state = MOTOR_STATE_IDLE;
static motor_error_t current_error = MOTOR_ERROR_NONE;

/* Calibration data */
typedef struct {
    bool is_valid;
    int32_t total_steps;
    int32_t backoff_steps;
    int8_t sgt_threshold;
} calibration_t;

static calibration_t calibration = {
    .is_valid = false,
    .total_steps = 0,
    .backoff_steps = DEFAULT_BACKOFF_STEPS,
    .sgt_threshold = -6
};

static bool is_homed = false;

/* ============================================
   TIMER CALLBACK
   ============================================ */
static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer, 
                                          const gptimer_alarm_event_data_t *edata, 
                                          void *user_ctx)
{
    if (steps_remaining > 0 || steps_remaining == -1) {
        step_state = !step_state;
        gpio_set_level(PIN_MOTOR_STEP, step_state);
        
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

/* ============================================
   PRIVATE FUNCTIONS
   ============================================ */
static esp_err_t set_speed_rpm(uint32_t rpm)
{
    if (rpm == 0) return ESP_ERR_INVALID_ARG;
    
    uint32_t steps_per_second = (rpm * STEPS_PER_REV * MOTOR_MICROSTEPS) / 60;
    uint32_t interval_us = 500000 / steps_per_second;
    if (interval_us < 10) interval_us = 10;
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = interval_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    
    return gptimer_set_alarm_action(step_timer, &alarm_config);
}

static void set_direction(bool forward)
{
    direction_forward = forward;
    gpio_set_level(PIN_MOTOR_DIR, forward ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void start_continuous_movement(bool forward)
{
    set_direction(forward);
    steps_remaining = -1;
    motor_running = true;
    gptimer_set_raw_count(step_timer, 0);
    gptimer_start(step_timer);
}

static void move_steps_blocking(int32_t steps)
{
    if (steps == 0) return;
    
    set_direction(steps > 0);
    steps_remaining = (steps > 0) ? steps : -steps;
    motor_running = true;
    current_state = MOTOR_STATE_MOVING;
    
    gptimer_set_raw_count(step_timer, 0);
    gptimer_start(step_timer);
    
    while (motor_running) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    current_state = MOTOR_STATE_IDLE;
}

static bool check_stall(void)
{
    static int64_t last_stall_time = 0;
    
    uint16_t sg_result = tmc2130_get_sg_result(&tmc2130);
    bool stall_flag = tmc2130_is_stallguard(&tmc2130);
    
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

static void configure_for_homing(void)
{
    tmc2130_set_en_pwm_mode(&tmc2130, false);
    tmc2130_set_toff(&tmc2130, 4);
    tmc2130_set_hstrt(&tmc2130, 4);
    tmc2130_set_hend(&tmc2130, 1);
    tmc2130_set_tbl(&tmc2130, 2);
    tmc2130_set_chm(&tmc2130, false);
    tmc2130_set_tcoolthrs(&tmc2130, 0xFFFFF);
    tmc2130_set_sgt(&tmc2130, calibration.sgt_threshold);
    tmc2130_set_sfilt(&tmc2130, true);
    tmc2130_set_diag1_stall(&tmc2130, true);
    tmc2130_set_diag1_pushpull(&tmc2130, true);
}

static void configure_for_normal_operation(void)
{
    tmc2130_set_en_pwm_mode(&tmc2130, true);
    tmc2130_set_pwm_autoscale(&tmc2130, true);
    tmc2130_set_pwm_freq(&tmc2130, 1);
    tmc2130_set_pwm_ampl(&tmc2130, 255);
    tmc2130_set_pwm_grad(&tmc2130, 5);
    tmc2130_set_diag1_stall(&tmc2130, false);
}

static int32_t find_endpoint(bool forward)
{
    int32_t start_position = current_position;
    
    set_speed_rpm(HOMING_SPEED_RPM);
    start_continuous_movement(forward);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while (motor_running) {
        if (check_stall()) {
            motor_stop();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    int32_t steps_moved = current_position - start_position;
    if (steps_moved < 0) steps_moved = -steps_moved;
    
    return steps_moved;
}

/* ============================================
   PUBLIC FUNCTIONS - INITIALIZATION
   ============================================ */
esp_err_t motor_init(void)
{
    ESP_LOGI(TAG, "Initializing motor control...");
    
    /* Initialize step timer */
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
    
    /* Initialize TMC2130 */
    tmc2130_config_t tmc_config = {
        .spi_host = SPI2_HOST,
        .pin_mosi = PIN_SPI_MOSI,
        .pin_miso = PIN_SPI_MISO,
        .pin_sclk = PIN_SPI_SCLK,
        .pin_cs = PIN_MOTOR_CS,
        .pin_en = PIN_MOTOR_EN,
        .pin_step = PIN_MOTOR_STEP,
        .pin_dir = PIN_MOTOR_DIR,
        .spi_clock_speed_hz = 1000000,
        .r_sense = MOTOR_R_SENSE,
    };
    
    esp_err_t ret = tmc2130_init(&tmc2130, &tmc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TMC2130 init failed: %s", esp_err_to_name(ret));
        current_error = MOTOR_ERROR_DRIVER_FAULT;
        return ret;
    }
    
    /* Configure TMC2130 */
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
    
    tmc2130_enable(&tmc2130);
    configure_for_normal_operation();
    set_speed_rpm(NORMAL_SPEED_RPM);
    
    ESP_LOGI(TAG, "Motor control initialized");
    return ESP_OK;
}

esp_err_t motor_load_calibration(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No calibration found in NVS");
        return ret;
    }
    
    uint8_t valid = 0;
    ret = nvs_get_u8(nvs_handle, NVS_KEY_CALIBRATED, &valid);
    if (ret != ESP_OK || valid == 0) {
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    calibration.is_valid = true;
    nvs_get_i32(nvs_handle, NVS_KEY_TOTAL_STEPS, &calibration.total_steps);
    nvs_get_i32(nvs_handle, NVS_KEY_BACKOFF, &calibration.backoff_steps);
    nvs_get_i8(nvs_handle, NVS_KEY_SGT, &calibration.sgt_threshold);
    
    nvs_close(nvs_handle);
    
    tmc2130_set_sgt(&tmc2130, calibration.sgt_threshold);
    
    ESP_LOGI(TAG, "Calibration loaded: %ld steps, SGT=%d", 
             (long)calibration.total_steps, calibration.sgt_threshold);
    
    return ESP_OK;
}

esp_err_t motor_save_calibration(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    nvs_set_u8(nvs_handle, NVS_KEY_CALIBRATED, calibration.is_valid ? 1 : 0);
    nvs_set_i32(nvs_handle, NVS_KEY_TOTAL_STEPS, calibration.total_steps);
    nvs_set_i32(nvs_handle, NVS_KEY_BACKOFF, calibration.backoff_steps);
    nvs_set_i8(nvs_handle, NVS_KEY_SGT, calibration.sgt_threshold);
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Calibration saved");
    return ret;
}

esp_err_t motor_clear_calibration(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    
    calibration.is_valid = false;
    is_homed = false;
    
    ESP_LOGI(TAG, "Calibration cleared");
    return ESP_OK;
}

/* ============================================
   PUBLIC FUNCTIONS - CALIBRATION & HOMING
   ============================================ */
esp_err_t motor_calibrate(void)
{
    ESP_LOGI(TAG, "Starting calibration...");
    
    current_state = MOTOR_STATE_CALIBRATING;
    is_homed = false;
    calibration.is_valid = false;
    
    configure_for_homing();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* Find first endpoint */
    find_endpoint(false);
    move_steps_blocking(calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(200));
    find_endpoint(false);
    
    current_position = 0;
    
    move_steps_blocking(calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* Find second endpoint */
    find_endpoint(true);
    int32_t max_pos = current_position;
    
    move_steps_blocking(-calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(200));
    find_endpoint(true);
    max_pos = current_position;
    
    /* Save calibration */
    calibration.total_steps = max_pos;
    calibration.is_valid = true;
    is_homed = true;
    
    move_steps_blocking(-calibration.backoff_steps);
    
    motor_save_calibration();
    
    configure_for_normal_operation();
    set_speed_rpm(NORMAL_SPEED_RPM);
    current_state = MOTOR_STATE_IDLE;
    
    ESP_LOGI(TAG, "Calibration complete: %ld steps", (long)calibration.total_steps);
    return ESP_OK;
}

esp_err_t motor_calibrate_with_sg_monitor(void)
{
    extern void console_printf(const char *fmt, ...);
    extern bool console_has_input(void);
    
    ESP_LOGI(TAG, "Starting calibration with SG monitoring...");
    console_printf("=== CALIBRATION WITH STALLGUARD MONITORING ===\r\n");
    
    current_state = MOTOR_STATE_CALIBRATING;
    is_homed = false;
    calibration.is_valid = false;
    
    configure_for_homing();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* Find first endpoint with monitoring */
    console_printf("Finding first endpoint...\r\n");
    set_speed_rpm(HOMING_SPEED_RPM);
    start_continuous_movement(false);
    
    bool stall_detected = false;
    uint16_t last_sg = 0;
    while (motor_running && !stall_detected) {
        uint16_t sg_val = tmc2130_get_sg_result(&tmc2130);
        if (sg_val != last_sg) {
            console_printf("SG: %4u\r\n", sg_val);
            last_sg = sg_val;
        }
        
        if (check_stall()) {
            motor_running = false;
            gptimer_stop(step_timer);
            stall_detected = true;
            console_printf("*** STALL DETECTED at SG=%u ***\r\n", sg_val);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    move_steps_blocking(calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    console_printf("\r\nPress any key to continue or wait 3 seconds...\r\n");
    for (int i = 0; i < 30; i++) {
        if (console_has_input()) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    console_printf("Finding first endpoint (precise)...\r\n");
    start_continuous_movement(false);
    last_sg = 0;
    while (motor_running && !check_stall()) {
        uint16_t sg_val = tmc2130_get_sg_result(&tmc2130);
        if (sg_val != last_sg) {
            console_printf("SG: %4u\r\n", sg_val);
            last_sg = sg_val;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motor_running = false;
    gptimer_stop(step_timer);
    
    current_position = 0;
    move_steps_blocking(calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* Find second endpoint with monitoring */
    console_printf("\r\nFinding second endpoint...\r\n");
    start_continuous_movement(true);
    last_sg = 0;
    while (motor_running && !check_stall()) {
        uint16_t sg_val = tmc2130_get_sg_result(&tmc2130);
        if (sg_val != last_sg) {
            console_printf("SG: %4u\r\n", sg_val);
            last_sg = sg_val;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motor_running = false;
    gptimer_stop(step_timer);
    
    int32_t max_pos = current_position;
    move_steps_blocking(-calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    console_printf("\r\nFinding second endpoint (precise)...\r\n");
    start_continuous_movement(true);
    last_sg = 0;
    while (motor_running && !check_stall()) {
        uint16_t sg_val = tmc2130_get_sg_result(&tmc2130);
        if (sg_val != last_sg) {
            console_printf("SG: %4u\r\n", sg_val);
            last_sg = sg_val;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motor_running = false;
    gptimer_stop(step_timer);
    
    max_pos = current_position;
    
    /* Save calibration */
    calibration.total_steps = max_pos;
    calibration.is_valid = true;
    is_homed = true;
    
    move_steps_blocking(-calibration.backoff_steps);
    
    motor_save_calibration();
    
    configure_for_normal_operation();
    set_speed_rpm(NORMAL_SPEED_RPM);
    current_state = MOTOR_STATE_IDLE;
    
    console_printf("\r\n=== CALIBRATION COMPLETE ===\r\n");
    console_printf("Total steps: %ld\r\n", (long)calibration.total_steps);
    console_printf("SGT threshold: %d\r\n", calibration.sgt_threshold);
    console_printf("============================\r\n\r\n");
    
    ESP_LOGI(TAG, "Calibration complete: %ld steps", (long)calibration.total_steps);
    return ESP_OK;
}

esp_err_t motor_home(void)
{
    if (!calibration.is_valid) {
        ESP_LOGW(TAG, "Cannot home - not calibrated");
        current_error = MOTOR_ERROR_NOT_CALIBRATED;
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting fast home...");
    current_state = MOTOR_STATE_HOMING;
    
    configure_for_homing();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    find_endpoint(false);
    move_steps_blocking(calibration.backoff_steps);
    vTaskDelay(pdMS_TO_TICKS(200));
    find_endpoint(false);
    
    current_position = 0;
    is_homed = true;
    
    move_steps_blocking(calibration.backoff_steps);
    
    configure_for_normal_operation();
    set_speed_rpm(NORMAL_SPEED_RPM);
    current_state = MOTOR_STATE_IDLE;
    
    ESP_LOGI(TAG, "Homing complete");
    return ESP_OK;
}

/* ============================================
   PUBLIC FUNCTIONS - MOVEMENT
   ============================================ */
esp_err_t motor_move_to_percent(float percent)
{
    if (!is_homed) {
        current_error = MOTOR_ERROR_NOT_HOMED;
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!calibration.is_valid) {
        current_error = MOTOR_ERROR_NOT_CALIBRATED;
        return ESP_ERR_INVALID_STATE;
    }
    
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    int32_t usable_min = calibration.backoff_steps;
    int32_t usable_max = calibration.total_steps - calibration.backoff_steps;
    int32_t usable_range = usable_max - usable_min;
    
    int32_t target = usable_min + (int32_t)((float)usable_range * percent / 100.0f);
    
    return motor_move_to_position(target);
}

esp_err_t motor_move_to_position(int32_t target)
{
    if (!is_homed) {
        current_error = MOTOR_ERROR_NOT_HOMED;
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Clamp to safe range */
    int32_t safe_min = calibration.backoff_steps;
    int32_t safe_max = calibration.total_steps - calibration.backoff_steps;
    
    if (target < safe_min) target = safe_min;
    if (target > safe_max) target = safe_max;
    
    int32_t steps_to_move = target - current_position;
    
    if (steps_to_move != 0) {
        ESP_LOGI(TAG, "Moving to %ld (delta: %ld)", (long)target, (long)steps_to_move);
        move_steps_blocking(steps_to_move);
    }
    
    return ESP_OK;
}

void motor_stop(void)
{
    steps_remaining = 0;
    motor_running = false;
    gptimer_stop(step_timer);
    gpio_set_level(PIN_MOTOR_STEP, 0);
    current_state = MOTOR_STATE_IDLE;
}

void motor_enable(void)
{
    tmc2130_enable(&tmc2130);
}

void motor_disable(void)
{
    motor_stop();
    tmc2130_disable(&tmc2130);
}

/* ============================================
   PUBLIC FUNCTIONS - CONFIGURATION
   ============================================ */
void motor_set_sgt(int8_t threshold)
{
    if (threshold < -64) threshold = -64;
    if (threshold > 63) threshold = 63;
    
    calibration.sgt_threshold = threshold;
    tmc2130_set_sgt(&tmc2130, threshold);
}

int8_t motor_get_sgt(void)
{
    return calibration.sgt_threshold;
}

int8_t motor_increase_sgt(void)
{
    if (calibration.sgt_threshold < 63) {
        calibration.sgt_threshold++;
        tmc2130_set_sgt(&tmc2130, calibration.sgt_threshold);
    }
    return calibration.sgt_threshold;
}

int8_t motor_decrease_sgt(void)
{
    if (calibration.sgt_threshold > -64) {
        calibration.sgt_threshold--;
        tmc2130_set_sgt(&tmc2130, calibration.sgt_threshold);
    }
    return calibration.sgt_threshold;
}

/* ============================================
   PUBLIC FUNCTIONS - STATUS
   ============================================ */
void motor_get_status(motor_status_t *status)
{
    status->state = current_state;
    status->error = current_error;
    status->is_calibrated = calibration.is_valid;
    status->is_homed = is_homed;
    status->position_steps = current_position;
    status->position_percent = motor_get_position_percent();
    status->total_steps = calibration.total_steps;
    status->sgt_threshold = calibration.sgt_threshold;
}

bool motor_is_moving(void)
{
    return motor_running;
}

float motor_get_position_percent(void)
{
    if (!calibration.is_valid || calibration.total_steps == 0) {
        return -1.0f;
    }
    
    int32_t usable_min = calibration.backoff_steps;
    int32_t usable_max = calibration.total_steps - calibration.backoff_steps;
    int32_t usable_range = usable_max - usable_min;
    
    if (usable_range <= 0) return 0.0f;
    
    float percent = ((float)(current_position - usable_min) / (float)usable_range) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    return percent;
}

int32_t motor_get_position_steps(void)
{
    return current_position;
}

/* ============================================
   PUBLIC FUNCTIONS - DIAGNOSTICS
   ============================================ */
uint16_t motor_read_stallguard(void)
{
    return tmc2130_get_sg_result(&tmc2130);
}

bool motor_test_connection(void)
{
    return tmc2130_test_connection(&tmc2130);
}