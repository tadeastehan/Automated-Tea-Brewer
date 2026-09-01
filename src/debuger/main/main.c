/**
 * @file main.c
 * @brief Stepper Motor TMC2130 & D8563TS RTC Debugger and Diagnostic Tool
 * 
 * Interactive debugger for TMC2130 stepper motor driver and D8563TS / PCF8563 I2C RTC.
 * Compatible with ESP32, ESP32-C6, ESP32-S3 via USB Serial JTAG / UART Console.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/soc_caps.h"

#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include "driver/usb_serial_jtag.h"
#endif

#include "main_pins.h"
#include "tmc2130.h"
#include "motor_control.h"
#include "d8563.h"
#include "distance_sensor.h"
#include "mlx90614.h"

static const char *TAG = "DEBUGGER";

#define RX_BUF_SIZE 512

/* ============================================================================
   CONSOLE I/O IMPLEMENTATION (Exclusively USB CDC / USB Serial JTAG)
   ============================================================================ */

#if SOC_USB_SERIAL_JTAG_SUPPORTED
static bool usb_serial_initialized = false;
#endif

esp_err_t console_init(void)
{
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = RX_BUF_SIZE,
    };
    if (usb_serial_jtag_driver_install(&config) == ESP_OK) {
        usb_serial_initialized = true;
    }
#endif
    return ESP_OK;
}

void console_print(const char *str)
{
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    if (usb_serial_initialized) {
        usb_serial_jtag_write_bytes((const uint8_t *)str, strlen(str), pdMS_TO_TICKS(100));
    }
#endif
}

void console_printf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    console_print(buf);
}

int console_getchar_timeout(uint32_t timeout_ms)
{
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    if (usb_serial_initialized) {
        uint8_t c;
        int len = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(timeout_ms));
        if (len > 0) return (int)c;
    }
#endif
    return -1;
}

int console_getchar(void)
{
    return console_getchar_timeout(0);
}

bool console_has_input(void)
{
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    if (usb_serial_initialized) {
        uint8_t c;
        int len = usb_serial_jtag_read_bytes(&c, 1, 0);
        if (len > 0) return true;
    }
#endif
    return false;
}

/* ============================================================================
   TMC2130 DIAGNOSTIC & REGISTER DECODING FUNCTIONS
   ============================================================================ */

/**
 * @brief Print full detailed status from DRV_STATUS register
 */
static void print_drv_status_details(void)
{
    uint16_t sg = motor_read_stallguard();
    motor_status_t st;
    motor_get_status(&st);

    console_printf("\r\n================ TMC2130 DRV_STATUS ================\r\n");
    console_printf("  Driver Connection : %s\r\n", motor_test_connection() ? "ONLINE (OK)" : "OFFLINE / NO RESPONSE");
    console_printf("  StallGuard Result : %u / 1023\r\n", sg);
    console_printf("  SGT Threshold     : %d\r\n", st.sgt_threshold);
    console_printf("  Position          : %ld steps (%.1f%%)\r\n", (long)st.position_steps, st.position_percent);
    console_printf("  Total Range       : %ld steps\r\n", (long)st.total_steps);
    console_printf("  Calibrated / Homed: %s / %s\r\n", st.is_calibrated ? "YES" : "NO", st.is_homed ? "YES" : "NO");
    console_printf("  Current State     : %s\r\n", 
                   st.state == MOTOR_STATE_IDLE ? "IDLE" :
                   st.state == MOTOR_STATE_MOVING ? "MOVING" :
                   st.state == MOTOR_STATE_HOMING ? "HOMING" :
                   st.state == MOTOR_STATE_CALIBRATING ? "CALIBRATING" : "ERROR");

    if (st.error != MOTOR_ERROR_NONE) {
        console_printf("  Active Error      : %s\r\n",
                       st.error == MOTOR_ERROR_NOT_CALIBRATED ? "NOT CALIBRATED" :
                       st.error == MOTOR_ERROR_NOT_HOMED ? "NOT HOMED" :
                       st.error == MOTOR_ERROR_STALL_DETECTED ? "STALL DETECTED" :
                       st.error == MOTOR_ERROR_DRIVER_FAULT ? "DRIVER FAULT" :
                       st.error == MOTOR_ERROR_OVERTEMP ? "OVER-TEMPERATURE" : "UNKNOWN");
    } else {
        console_printf("  Active Error      : NONE\r\n");
    }
    console_printf("====================================================\r\n\r\n");
}

/**
 * @brief Live monitor task: streams StallGuard and status until user presses a key
 */
static void run_live_monitor(void)
{
    console_printf("\r\n--- LIVE MOTOR TELEMETRY (Press any key to exit) ---\r\n");
    console_printf("   Time(s) | Steps   |  Percent | StallGuard | Moving\r\n");
    console_printf("----------------------------------------------------\r\n");

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        if (console_getchar() >= 0) {
            break;
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000) - start_ms;
        uint16_t sg = motor_read_stallguard();
        int32_t pos = motor_get_position_steps();
        float pct = motor_get_position_percent();
        bool moving = motor_is_moving();

        console_printf("  %6.2f s | %7ld | %7.1f%% |    %4u    | %s\r\n",
                       (float)now_ms / 1000.0f,
                       (long)pos,
                       pct >= 0 ? pct : 0.0f,
                       sg,
                       moving ? "YES" : "NO");

        vTaskDelay(pdMS_TO_TICKS(150));
    }
    console_printf("--- Monitor stopped ---\r\n\r\n");
}

/**
 * @brief Automated Stepper Motor Self-Test Routine
 */
static void run_self_test(void)
{
    console_printf("\r\n================ RUNNING MOTOR SELF TEST ================\r\n");
    
    // Step 1: SPI connection
    console_printf("[1/4] Checking TMC2130 SPI connection... ");
    bool connected = motor_test_connection();
    if (connected) {
        console_printf("PASS [OK]\r\n");
    } else {
        console_printf("FAIL [SPI NOT RESPONDING]\r\n");
        console_printf("      Check wiring: MOSI=%d, MISO=%d, SCLK=%d, CS=%d\r\n",
                       PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK, PIN_MOTOR_CS);
        return;
    }

    // Step 2: StallGuard reading test
    console_printf("[2/4] Reading StallGuard value... ");
    uint16_t sg = motor_read_stallguard();
    console_printf("PASS (Value: %u)\r\n", sg);

    // Step 3: Enable driver
    console_printf("[3/4] Enabling motor driver coil output... ");
    motor_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    console_printf("PASS [OK]\r\n");

    // Step 4: Small forward and backward nudge
    console_printf("[4/4] Performing small step nudge (forward & back)... ");
    for (int i = 0; i < 200; i++) {
        gpio_set_level(PIN_MOTOR_DIR, 1);
        gpio_set_level(PIN_MOTOR_STEP, 1);
        esp_rom_delay_us(100);
        gpio_set_level(PIN_MOTOR_STEP, 0);
        esp_rom_delay_us(100);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    for (int i = 0; i < 200; i++) {
        gpio_set_level(PIN_MOTOR_DIR, 0);
        gpio_set_level(PIN_MOTOR_STEP, 1);
        esp_rom_delay_us(100);
        gpio_set_level(PIN_MOTOR_STEP, 0);
        esp_rom_delay_us(100);
    }
    console_printf("PASS [OK]\r\n");

    console_printf("================ SELF TEST COMPLETE ================\r\n\r\n");
}

/* ============================================================================
   D8563TS I2C RTC DIAGNOSTIC & TESTING FUNCTIONS
   ============================================================================ */

/**
 * @brief Run full D8563TS test & register dump
 */
static void run_d8563_test(void)
{
    gpio_num_t sda, scl;
    d8563_get_pins(&sda, &scl, NULL);

    console_printf("\r\n================ D8563TS I2C RTC TEST ================\r\n");
    console_printf("Target Address : 0x51 (SDA: GPIO %d, SCL: GPIO %d)\r\n", sda, scl);

    bool connected = d8563_is_connected();
    if (!connected) {
        console_printf("Result         : [FAILED] Device not responding at 0x51!\r\n\r\n");
        d8563_diagnose_bus();
        return;
    }

    console_printf("Result         : [PASS] D8563TS detected and responding!\r\n\r\n");

    d8563_status_t st;
    if (d8563_get_status(&st) == ESP_OK) {
        console_printf("--- Raw Register Dump (0x00 - 0x0F) ---\r\n");
        for (int i = 0; i < 16; i++) {
            console_printf("  [0x%02X] = 0x%02X", i, st.raw_regs[i]);
            if ((i + 1) % 4 == 0) console_printf("\r\n");
        }
        console_printf("\r\n");

        console_printf("--- Decoded Status & Time ---\r\n");
        console_printf("  Oscillator State : %s\r\n", st.is_running ? "RUNNING (STOP=0)" : "STOPPED (STOP=1)");
        console_printf("  Voltage Low (VL) : %s %s\r\n", 
                       st.voltage_low ? "YES (Power was lost / Battery low)" : "NO (Clock integrity guaranteed)",
                       st.voltage_low ? "[!] Run 'rtc now' or 'rtc set' to initialize time" : "");
        console_printf("  Current Time     : %04d-%02d-%02d %02d:%02d:%02d (%s)\r\n",
                       st.time.year, st.time.month, st.time.day,
                       st.time.hour, st.time.minute, st.time.second,
                       d8563_weekday_name(st.time.weekday));
        console_printf("  Control 1 / 2    : 0x%02X / 0x%02X\r\n", st.raw_regs[0], st.raw_regs[1]);
        console_printf("  CLKOUT Control   : 0x%02X\r\n", st.raw_regs[0x0D]);
        console_printf("  Timer Control    : 0x%02X (Val: 0x%02X)\r\n", st.raw_regs[0x0E], st.raw_regs[0x0F]);
    } else {
        console_printf("[ERROR] Failed to read registers from D8563!\r\n");
    }

    console_printf("======================================================\r\n\r\n");
}

/**
 * @brief Live RTC time stream to confirm 32.768 kHz oscillator ticking
 */
static void run_d8563_stream(void)
{
    if (!d8563_is_connected()) {
        console_printf("[ERROR] D8563TS not detected on I2C bus at 0x51!\r\n");
        return;
    }

    console_printf("\r\n--- LIVE D8563TS RTC STREAM (Press any key to exit) ---\r\n");
    console_printf(" Date       Time       Weekday    VL Flag   Status\r\n");
    console_printf("----------------------------------------------------\r\n");

    int last_sec = -1;
    while (1) {
        if (console_getchar() >= 0) {
            break;
        }

        d8563_time_t t;
        if (d8563_get_time(&t) == ESP_OK) {
            if (t.second != last_sec) {
                last_sec = t.second;
                console_printf(" %04d-%02d-%02d %02d:%02d:%02d  %-9s  VL=%d       Ticking OK\r\n",
                               t.year, t.month, t.day,
                               t.hour, t.minute, t.second,
                               d8563_weekday_name(t.weekday),
                               t.voltage_low ? 1 : 0);
            }
        } else {
            console_printf(" [ERROR reading RTC]\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    console_printf("--- RTC Stream stopped ---\r\n\r\n");
}

/* ============================================================================
   VL53L0X LASER DISTANCE SENSOR FUNCTIONS
   ============================================================================ */

static void run_distance_test(void)
{
    console_printf("\r\n================ VL53L0X LASER DISTANCE SENSOR TEST ================\r\n");
    console_printf("Target Address : 0x29 (SDA: GPIO %d, SCL: GPIO %d)\r\n", PIN_I2C_SDA, PIN_I2C_SCL);

    if (!distance_sensor_is_initialized()) {
        console_printf("Initializing VL53L0X sensor... ");
        esp_err_t err = distance_sensor_init(NULL);
        if (err != ESP_OK) {
            console_printf("FAIL (%s)\r\n", esp_err_to_name(err));
            console_printf("Check if sensor is connected to VCC, GND, SDA (GPIO %d), SCL (GPIO %d).\r\n\r\n",
                           PIN_I2C_SDA, PIN_I2C_SCL);
            return;
        }
        console_printf("OK [ONLINE]\r\n");
    }

    uint16_t dist_mm = 0;
    console_printf("Measuring distance... ");
    esp_err_t err = distance_sensor_get_distance(&dist_mm);
    if (err == ESP_OK) {
        float dist_cm = (float)dist_mm / 10.0f;
        float dist_in = (float)dist_mm / 25.4f;
        console_printf("SUCCESS!\r\n");
        console_printf("  Distance : %u mm  (%.1f cm / %.2f in)\r\n", dist_mm, dist_cm, dist_in);
        if (dist_mm >= 8190) {
            console_printf("  Status   : [OUT OF RANGE / NO TARGET]\r\n");
        } else {
            console_printf("  Status   : [TARGET DETECTED - VALID]\r\n");
        }
    } else {
        console_printf("FAIL (%s)\r\n", esp_err_to_name(err));
    }
    console_printf("====================================================================\r\n\r\n");
}

static void run_distance_stream(void)
{
    console_printf("\r\n--- Starting VL53L0X Live Distance Stream (Press any key or 'q' to stop) ---\r\n");

    if (!distance_sensor_is_initialized()) {
        console_printf("Initializing VL53L0X sensor... ");
        esp_err_t err = distance_sensor_init(NULL);
        if (err != ESP_OK) {
            console_printf("FAIL (%s)\r\n\r\n", esp_err_to_name(err));
            return;
        }
        console_printf("OK [ONLINE]\r\n");
    }

    console_printf("  Elapsed | Distance (mm) | Distance (cm) | Proximity Gauge\r\n");
    console_printf(" ---------+---------------+---------------+------------------------------------\r\n");

    int64_t start_us = esp_timer_get_time();
    while (1) {
        if (console_getchar() >= 0) {
            break;
        }

        uint16_t dist_mm = 0;
        esp_err_t err = distance_sensor_get_distance(&dist_mm);
        int64_t now_ms = (esp_timer_get_time() - start_us) / 1000;

        if (err == ESP_OK) {
            char bar[31];
            int bars = dist_mm / 25;
            if (bars > 30) bars = 30;
            for (int i = 0; i < bars; i++) bar[i] = '=';
            bar[bars] = '\0';

            if (dist_mm >= 8190) {
                console_printf("  %6.2f s |     OUT       |      OUT      | [OUT OF RANGE]\r\n", (float)now_ms / 1000.0f);
            } else {
                console_printf("  %6.2f s |   %5u mm     |   %6.1f cm   | [|%-30s]\r\n",
                               (float)now_ms / 1000.0f, dist_mm, (float)dist_mm / 10.0f, bar);
            }
        } else {
            console_printf("  %6.2f s | [READ ERROR: %s]\r\n", (float)now_ms / 1000.0f, esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    console_printf("--- Distance Stream stopped ---\r\n\r\n");
}

/* ============================================================================
   MLX90614 NON-CONTACT INFRARED THERMOMETER FUNCTIONS
   ============================================================================ */

static void run_thermometer_test(void)
{
    console_printf("\r\n================ MLX90614 IR THERMOMETER TEST ================\r\n");
    console_printf("Target Address : 0x5A (SDA: GPIO %d, SCL: GPIO %d)\r\n", PIN_I2C_SDA, PIN_I2C_SCL);

    mlx90614_readings_t readings;
    esp_err_t err = mlx90614_get_readings(&readings);
    if (err == ESP_OK) {
        float amb_f = (readings.ambient_c * 9.0f / 5.0f) + 32.0f;
        float obj_raw_f = (readings.object_raw_c * 9.0f / 5.0f) + 32.0f;
        float obj_calib_f = (readings.object_calib_c * 9.0f / 5.0f) + 32.0f;

        console_printf("Status         : [ONLINE] Communication OK\r\n");
        console_printf("  Ambient Temp : %6.1f °C  (%6.1f °F)\r\n", readings.ambient_c, amb_f);
        console_printf("  Object (Raw) : %6.1f °C  (%6.1f °F)\r\n", readings.object_raw_c, obj_raw_f);
        console_printf("  Object (Cal) : %6.1f °C  (%6.1f °F) [Calibrated for Tea Liquid]\r\n",
                       readings.object_calib_c, obj_calib_f);
    } else {
        console_printf("Status         : [FAILED] No response at 0x5A!\r\n");
        console_printf("Check if sensor is connected to VCC (3.3V), GND, SDA (GPIO %d), SCL (GPIO %d).\r\n",
                       PIN_I2C_SDA, PIN_I2C_SCL);
    }
    console_printf("==============================================================\r\n\r\n");
}

static void run_thermometer_stream(void)
{
    console_printf("\r\n--- Starting MLX90614 Live Temperature Stream (Press any key to exit) ---\r\n");
    console_printf("  Elapsed | Ambient (°C) | Raw IR (°C) | Calibrated (°C) | Thermal Gauge (0-100°C)\r\n");
    console_printf(" ---------+--------------+-------------+-----------------+------------------------------------\r\n");

    int64_t start_us = esp_timer_get_time();
    while (1) {
        if (console_getchar() >= 0) {
            break;
        }

        mlx90614_readings_t readings;
        esp_err_t err = mlx90614_get_readings(&readings);
        int64_t now_ms = (esp_timer_get_time() - start_us) / 1000;

        if (err == ESP_OK) {
            char bar[31];
            int bars = (int)(readings.object_calib_c / 3.33f);
            if (bars < 0) bars = 0;
            if (bars > 30) bars = 30;
            for (int i = 0; i < bars; i++) bar[i] = '#';
            bar[bars] = '\0';

            console_printf("  %6.2f s |    %5.1f °C  |   %5.1f °C  |     %5.1f °C    | [|%-30s]\r\n",
                           (float)now_ms / 1000.0f,
                           readings.ambient_c,
                           readings.object_raw_c,
                           readings.object_calib_c,
                           bar);
        } else {
            console_printf("  %6.2f s | [READ ERROR: %s]\r\n", (float)now_ms / 1000.0f, esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
    console_printf("--- Temperature Stream stopped ---\r\n\r\n");
}

/* ============================================================================
   INDUCTION COOKER CONTROL (PIN_INDUCTION = GPIO 3)
   ============================================================================ */

static bool s_induction_enabled = false;

static void induction_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_INDUCTION),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_INDUCTION, 0);
    s_induction_enabled = false;
}

static void induction_set(bool on)
{
    s_induction_enabled = on;
    gpio_set_level(PIN_INDUCTION, on ? 1 : 0);
    console_printf("[INDUCTION] Cooker Output: %s (GPIO %d = %d)\r\n",
                   on ? "ON [HEATING ENABLED]" : "OFF [DISABLED]", PIN_INDUCTION, on ? 1 : 0);
}

static void induction_toggle(void)
{
    induction_set(!s_induction_enabled);
}

static void induction_pulse(uint32_t pulse_ms)
{
    console_printf("[INDUCTION] Pulsing GPIO %d for %lu ms (Simulated button press)...\r\n", PIN_INDUCTION, (unsigned long)pulse_ms);
    gpio_set_level(PIN_INDUCTION, 1);
    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    gpio_set_level(PIN_INDUCTION, 0);
    s_induction_enabled = false;
    console_printf("[INDUCTION] Pulse complete (GPIO %d = 0)\r\n", PIN_INDUCTION);
}

/* ============================================================================
   COMMAND LINE INTERFACE & MENU SYSTEM
   ============================================================================ */

static void print_menu(void)
{
    gpio_num_t sda, scl;
    d8563_get_pins(&sda, &scl, NULL);

    console_printf("\r\n");
    console_printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    console_printf("║     TEA BREWER HARDWARE DEBUGGER (MOTOR, RTC, TOF & IR)      ║\r\n");
    console_printf("╠══════════════════════════════════════════════════════════════╣\r\n");
    console_printf("║ [INDUCTION COOKER CONTROL]                                   ║\r\n");
    console_printf("║   ind on / ind off - Enable / Disable induction heater (GPIO %-2d)║\r\n", PIN_INDUCTION);
    console_printf("║   ind / ind tog    - Toggle induction state (ON <-> OFF)     ║\r\n");
    console_printf("║   ind pulse [ms]   - Momentary button pulse (default 200 ms) ║\r\n");
    console_printf("║                                                              ║\r\n");
    console_printf("║ [MLX90614 IR TEMPERATURE SENSOR]                            ║\r\n");
    console_printf("║   temp / ir     - Single non-contact IR temperature test     ║\r\n");
    console_printf("║   temp mon      - Live real-time thermal telemetry stream    ║\r\n");
    console_printf("║                                                              ║\r\n");
    console_printf("║ [VL53L0X LASER DISTANCE SENSOR]                              ║\r\n");
    console_printf("║   dist / tof    - Single laser distance measurement (mm)     ║\r\n");
    console_printf("║   dist mon      - Live real-time distance telemetry stream   ║\r\n");
    console_printf("║                                                              ║\r\n");
    console_printf("║ [D8563TS I2C RTC COMMANDS]                                   ║\r\n");
    console_printf("║   scan          - Scan I2C bus on GPIO %-2d (SDA) / %-2d (SCL)  ║\r\n", sda, scl);
    console_printf("║   scan <sda> <scl> - Scan with custom SDA/SCL pins           ║\r\n");
    console_printf("║   i2c diag      - Test physical SDA/SCL pull-up voltages     ║\r\n");
    console_printf("║   rtc / rtctest - Test D8563TS chip & dump all registers     ║\r\n");
    console_printf("║   rtc mon       - Live 1-second ticking stream test          ║\r\n");
    console_printf("║   rtc set <ts | Y M D h m s> - Set time (Unix ts or custom)  ║\r\n");
    console_printf("║                                                              ║\r\n");
    console_printf("║ [STEPPER MOTOR COMMANDS]                                     ║\r\n");
    console_printf("║   c / cal       - Full calibration (find endpoints, NO LOAD) ║\r\n");
    console_printf("║   t / calmon    - Calibration with live StallGuard monitor   ║\r\n");
    console_printf("║   f / home      - Fast homing (to start endpoint)            ║\r\n");
    console_printf("║   <0-100>       - Move to target percentage (e.g. '50')      ║\r\n");
    console_printf("║   pos <steps>   - Move to absolute step position             ║\r\n");
    console_printf("║   step <n> [us] - Step N raw steps (e.g. 'step 400 500')     ║\r\n");
    console_printf("║   x / stop      - Emergency stop & hold                      ║\r\n");
    console_printf("║   e / enable    - Enable motor driver coils                  ║\r\n");
    console_printf("║   d / disable   - Disable motor driver (freewheel)           ║\r\n");
    console_printf("║                                                              ║\r\n");
    console_printf("║ [DIAGNOSTICS & TUNING]                                       ║\r\n");
    console_printf("║   s / status    - Print full status for motor & RTC          ║\r\n");
    console_printf("║   m / monitor   - Live real-time StallGuard telemetry stream ║\r\n");
    console_printf("║   g / sg        - Single StallGuard reading                  ║\r\n");
    console_printf("║   sgt <val>     - Set StallGuard threshold (-64 to +63)      ║\r\n");
    console_printf("║   + / -         - Increase / Decrease SGT threshold          ║\r\n");
    console_printf("║   test          - Run automated hardware self-test           ║\r\n");
    console_printf("║                                                              ║\r\n");
    console_printf("║ [CALIBRATION STORAGE (NVS)]                                  ║\r\n");
    console_printf("║   w / save      - Save calibration to NVS Flash              ║\r\n");
    console_printf("║   l / load      - Load calibration from NVS Flash            ║\r\n");
    console_printf("║   z / clear     - Clear calibration from NVS Flash           ║\r\n");
    console_printf("║   h / help      - Show this menu                             ║\r\n");
    console_printf("╚══════════════════════════════════════════════════════════════╝\r\n\r\n");
}

static void process_command_line(char *cmd)
{
    // Trim leading whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd == '\0') return;

    // Trim trailing whitespace / newlines
    int len = strlen(cmd);
    while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\r' || cmd[len - 1] == '\n' || cmd[len - 1] == '\t')) {
        cmd[--len] = '\0';
    }

    if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_menu();
        return;
    }

    /* ================= D8563TS RTC & I2C COMMANDS ================= */
    if (strcmp(cmd, "scan bb") == 0 || strcmp(cmd, "bb") == 0 || strcmp(cmd, "bbscan") == 0) {
        d8563_bitbang_scan();
        return;
    }

    if (strncmp(cmd, "ping", 4) == 0) {
        int addr = 0x51;
        sscanf(cmd + 4, "%x", &addr);
        d8563_bitbang_ping((uint8_t)addr);
        return;
    }

    if (strncmp(cmd, "scan", 4) == 0 || strncmp(cmd, "i2c pins", 8) == 0) {
        int sda_in = -1, scl_in = -1, spd_in = 50000;
        if (sscanf(cmd, "scan %d %d %d", &sda_in, &scl_in, &spd_in) >= 2 ||
            sscanf(cmd, "i2c pins %d %d %d", &sda_in, &scl_in, &spd_in) >= 2) {
            console_printf("Re-configuring I2C bus to SDA=GPIO %d, SCL=GPIO %d @ %d Hz...\r\n", sda_in, scl_in, spd_in);
            d8563_reinit_pins((gpio_num_t)sda_in, (gpio_num_t)scl_in, (uint32_t)spd_in);
        }
        d8563_scan_i2c_bus();
        return;
    }

    if (strcmp(cmd, "i2c") == 0) {
        d8563_scan_i2c_bus();
        return;
    }

    if (strcmp(cmd, "i2c diag") == 0 || strcmp(cmd, "diag") == 0) {
        d8563_diagnose_bus();
        return;
    }

    if (strcmp(cmd, "rtc") == 0 || strcmp(cmd, "rtctest") == 0 || strcmp(cmd, "rtc test") == 0) {
        run_d8563_test();
        return;
    }

    if (strcmp(cmd, "rtc mon") == 0 || strcmp(cmd, "rtc stream") == 0) {
        run_d8563_stream();
        return;
    }

    if (strncmp(cmd, "rtc set", 7) == 0) {
        char *arg = cmd + 7;
        while (*arg == ' ') arg++;

        int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
        int parsed = sscanf(arg, "%d %d %d %d %d %d", &y, &mo, &d, &h, &mi, &s);

        if (parsed == 6) {
            // Calculate day of week using Zeller / standard struct tm
            struct tm temp_tm = {
                .tm_year = y - 1900,
                .tm_mon  = mo - 1,
                .tm_mday = d,
                .tm_hour = h,
                .tm_min  = mi,
                .tm_sec  = s,
                .tm_isdst = -1,
            };
            mktime(&temp_tm);

            d8563_time_t t = {
                .year = y,
                .month = mo,
                .day = d,
                .weekday = temp_tm.tm_wday,
                .hour = h,
                .minute = mi,
                .second = s,
                .voltage_low = false
            };
            console_printf("Setting D8563TS time to %04d-%02d-%02d %02d:%02d:%02d (%s)...\r\n", 
                           y, mo, d, h, mi, s, d8563_weekday_name(t.weekday));
            esp_err_t ret = d8563_set_time(&t);
            if (ret == ESP_OK) {
                console_printf("[OK] Time set successfully!\r\n");
            } else {
                console_printf("[ERROR] Failed to set time: %s\r\n", esp_err_to_name(ret));
            }
        } else if (parsed == 1 && atoll(arg) > 946684800LL) {
            // Unix Timestamp (seconds since Jan 1 1970 UTC)
            int64_t raw_ts = (int64_t)atoll(arg);
            time_t ts = (time_t)raw_ts;
            struct tm tm_info;
            gmtime_r(&ts, &tm_info);

            d8563_time_t t = {
                .year = tm_info.tm_year + 1900,
                .month = tm_info.tm_mon + 1,
                .day = tm_info.tm_mday,
                .weekday = tm_info.tm_wday,
                .hour = tm_info.tm_hour,
                .minute = tm_info.tm_min,
                .second = tm_info.tm_sec,
                .voltage_low = false
            };
            console_printf("Setting D8563TS from Unix timestamp %lld -> %04d-%02d-%02d %02d:%02d:%02d (%s)...\r\n",
                           (long long)raw_ts, t.year, t.month, t.day, t.hour, t.minute, t.second, d8563_weekday_name(t.weekday));
            esp_err_t ret = d8563_set_time(&t);
            if (ret == ESP_OK) {
                console_printf("[OK] RTC synced to Unix timestamp %lld successfully!\r\n", (long long)raw_ts);
            } else {
                console_printf("[ERROR] Failed to set time: %s\r\n", esp_err_to_name(ret));
            }
        } else {
            console_printf("Usage:\r\n");
            console_printf("  rtc set <timestamp>              - e.g. 'rtc set 1725134400'\r\n");
            console_printf("  rtc set <Y> <M> <D> <h> <m> <s>  - e.g. 'rtc set 2026 8 31 21 30 00'\r\n");
        }
        return;
    }

    /* ================= MLX90614 IR THERMOMETER COMMANDS ================= */
    if (strcmp(cmd, "temp mon") == 0 || strcmp(cmd, "ir mon") == 0 || strcmp(cmd, "temp stream") == 0 || strcmp(cmd, "mlx mon") == 0) {
        run_thermometer_stream();
        return;
    }

    if (strcmp(cmd, "temp") == 0 || strcmp(cmd, "ir") == 0 || strcmp(cmd, "mlx") == 0 || strcmp(cmd, "thermo") == 0 || strcmp(cmd, "temp test") == 0) {
        run_thermometer_test();
        return;
    }

    /* ================= VL53L0X LASER DISTANCE SENSOR COMMANDS ================= */
    if (strcmp(cmd, "dist mon") == 0 || strcmp(cmd, "tof mon") == 0 || strcmp(cmd, "dist stream") == 0) {
        run_distance_stream();
        return;
    }

    if (strcmp(cmd, "dist") == 0 || strcmp(cmd, "tof") == 0 || strcmp(cmd, "laser") == 0 || strcmp(cmd, "dist test") == 0) {
        run_distance_test();
        return;
    }

    /* ================= INDUCTION COOKER COMMANDS ================= */
    if (strcmp(cmd, "ind on") == 0 || strcmp(cmd, "cooker on") == 0 || strcmp(cmd, "heat on") == 0) {
        induction_set(true);
        return;
    }

    if (strcmp(cmd, "ind off") == 0 || strcmp(cmd, "cooker off") == 0 || strcmp(cmd, "heat off") == 0) {
        induction_set(false);
        return;
    }

    if (strcmp(cmd, "ind") == 0 || strcmp(cmd, "ind tog") == 0 || strcmp(cmd, "ind toggle") == 0 || strcmp(cmd, "cooker") == 0) {
        induction_toggle();
        return;
    }

    if (strncmp(cmd, "ind pulse", 9) == 0 || strncmp(cmd, "pulse", 5) == 0) {
        char *arg = (strncmp(cmd, "ind pulse", 9) == 0) ? (cmd + 9) : (cmd + 5);
        while (*arg == ' ') arg++;
        uint32_t ms = 500;
        if (*arg != '\0') {
            ms = (uint32_t)atoi(arg);
            if (ms < 10) ms = 10;
        }
        induction_pulse(ms);
        return;
    }

    /* ================= MOTOR COMMANDS ================= */
    if (strcmp(cmd, "s") == 0 || strcmp(cmd, "status") == 0) {
        print_drv_status_details();
        run_d8563_test();
        run_distance_test();
        run_thermometer_test();
        console_printf("\r\n================ INDUCTION COOKER STATUS ================\r\n");
        console_printf("  Induction Output : %s (GPIO %d = %d)\r\n",
                       s_induction_enabled ? "ON [HEATING ACTIVE]" : "OFF [DISABLED]",
                       PIN_INDUCTION, s_induction_enabled ? 1 : 0);
        console_printf("=========================================================\r\n\r\n");
        return;
    }

    if (strcmp(cmd, "m") == 0 || strcmp(cmd, "monitor") == 0) {
        run_live_monitor();
        return;
    }

    if (strcmp(cmd, "test") == 0 || strcmp(cmd, "selftest") == 0) {
        run_self_test();
        run_d8563_test();
        run_distance_test();
        run_thermometer_test();
        return;
    }

    if (strcmp(cmd, "c") == 0 || strcmp(cmd, "cal") == 0 || strcmp(cmd, "calibrate") == 0) {
        console_printf("Starting full calibration (ensure no external load)...\r\n");
        esp_err_t ret = motor_calibrate();
        if (ret == ESP_OK) {
            console_printf("Calibration SUCCESSFUL!\r\n");
            print_drv_status_details();
        } else {
            console_printf("Calibration FAILED with error: %s\r\n", esp_err_to_name(ret));
        }
        return;
    }

    if (strcmp(cmd, "t") == 0 || strcmp(cmd, "calmon") == 0) {
        console_printf("Starting calibration with live StallGuard monitor...\r\n");
        esp_err_t ret = motor_calibrate_with_sg_monitor();
        if (ret == ESP_OK) {
            console_printf("Calibration with SG monitor SUCCESSFUL!\r\n");
        } else {
            console_printf("Calibration FAILED with error: %s\r\n", esp_err_to_name(ret));
        }
        return;
    }

    if (strcmp(cmd, "f") == 0 || strcmp(cmd, "home") == 0) {
        console_printf("Homing stepper motor...\r\n");
        esp_err_t ret = motor_home();
        if (ret == ESP_OK) {
            console_printf("Homing complete.\r\n");
        } else {
            console_printf("Homing FAILED (Is system calibrated? Run 'cal' or 'load')\r\n");
        }
        return;
    }

    if (strcmp(cmd, "g") == 0 || strcmp(cmd, "sg") == 0) {
        uint16_t sg = motor_read_stallguard();
        console_printf("Current StallGuard value: %u / 1023\r\n", sg);
        return;
    }

    if (strcmp(cmd, "+") == 0) {
        int8_t new_sgt = motor_increase_sgt();
        console_printf("Increased SGT (less sensitive) -> SGT = %d\r\n", new_sgt);
        return;
    }

    if (strcmp(cmd, "-") == 0) {
        int8_t new_sgt = motor_decrease_sgt();
        console_printf("Decreased SGT (more sensitive) -> SGT = %d\r\n", new_sgt);
        return;
    }

    if (strncmp(cmd, "sgt", 3) == 0) {
        char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        if (*arg != '\0') {
            int sgt = atoi(arg);
            motor_set_sgt((int8_t)sgt);
            console_printf("Set SGT threshold to %d\r\n", (int)motor_get_sgt());
        } else {
            console_printf("Current SGT threshold: %d (Usage: sgt <val> [-64..63])\r\n", (int)motor_get_sgt());
        }
        return;
    }

    if (strcmp(cmd, "e") == 0 || strcmp(cmd, "enable") == 0) {
        motor_enable();
        console_printf("Motor driver ENABLED.\r\n");
        return;
    }

    if (strcmp(cmd, "d") == 0 || strcmp(cmd, "disable") == 0) {
        motor_disable();
        console_printf("Motor driver DISABLED (coils free).\r\n");
        return;
    }

    if (strcmp(cmd, "x") == 0 || strcmp(cmd, "stop") == 0) {
        motor_stop();
        console_printf("Motor STOPPED immediately.\r\n");
        return;
    }

    if (strcmp(cmd, "w") == 0 || strcmp(cmd, "save") == 0) {
        esp_err_t ret = motor_save_calibration();
        if (ret == ESP_OK) {
            console_printf("Calibration saved to NVS Flash.\r\n");
        } else {
            console_printf("Failed to save calibration: %s\r\n", esp_err_to_name(ret));
        }
        return;
    }

    if (strcmp(cmd, "l") == 0 || strcmp(cmd, "load") == 0) {
        esp_err_t ret = motor_load_calibration();
        if (ret == ESP_OK) {
            console_printf("Calibration loaded from NVS Flash successfully.\r\n");
            print_drv_status_details();
        } else {
            console_printf("No calibration found in NVS Flash.\r\n");
        }
        return;
    }

    if (strcmp(cmd, "z") == 0 || strcmp(cmd, "clear") == 0) {
        motor_clear_calibration();
        console_printf("Calibration cleared from NVS Flash.\r\n");
        return;
    }

    // Step raw command: step <n_steps> [delay_us]
    if (strncmp(cmd, "step", 4) == 0) {
        char *arg = cmd + 4;
        while (*arg == ' ') arg++;
        if (*arg != '\0') {
            int steps = 0;
            int delay_us = 500;
            sscanf(arg, "%d %d", &steps, &delay_us);
            if (delay_us < 20) delay_us = 20;

            console_printf("Stepping %d steps (delay %d us)...\r\n", steps, delay_us);
            motor_enable();
            gpio_set_level(PIN_MOTOR_DIR, steps >= 0 ? 1 : 0);
            int count = abs(steps);
            for (int i = 0; i < count; i++) {
                gpio_set_level(PIN_MOTOR_STEP, 1);
                esp_rom_delay_us(delay_us);
                gpio_set_level(PIN_MOTOR_STEP, 0);
                esp_rom_delay_us(delay_us);
            }
            console_printf("Stepping complete.\r\n");
        } else {
            console_printf("Usage: step <num_steps> [delay_us]\r\n");
        }
        return;
    }

    // Pos command: pos <steps>
    if (strncmp(cmd, "pos", 3) == 0) {
        char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        if (*arg != '\0') {
            int32_t target_pos = (int32_t)atol(arg);
            console_printf("Moving to absolute position %ld steps...\r\n", (long)target_pos);
            esp_err_t ret = motor_move_to_position(target_pos);
            if (ret != ESP_OK) {
                console_printf("Move failed: %s (Must be calibrated and homed)\r\n", esp_err_to_name(ret));
            }
        } else {
            console_printf("Usage: pos <target_steps>\r\n");
        }
        return;
    }

    // Numerical move: 0 to 100 percent
    char *endptr;
    float pct = strtof(cmd, &endptr);
    if (endptr != cmd && *endptr == '\0') {
        if (pct >= 0.0f && pct <= 100.0f) {
            console_printf("Moving to %.1f%%...\r\n", pct);
            esp_err_t ret = motor_move_to_percent(pct);
            if (ret != ESP_OK) {
                console_printf("Move to percent failed: %s (Must be calibrated and homed)\r\n", esp_err_to_name(ret));
            }
        } else {
            console_printf("Percentage must be between 0.0 and 100.0\r\n");
        }
        return;
    }

    console_printf("Unknown command '%s'. Type 'h' or 'help' for command list.\r\n", cmd);
}

/* ============================================================================
 * CONSOLE INTERACTIVE TASK
 * ============================================================================ */
static void console_task(void *pvParameters)
{
    char line_buffer[128];
    int line_index = 0;

    print_menu();
    console_printf("debuger> ");

    while (1) {
        int c = console_getchar_timeout(50);
        if (c >= 0) {
            if (c == '\r' || c == '\n') {
                console_printf("\r\n");
                line_buffer[line_index] = '\0';
                if (line_index > 0) {
                    process_command_line(line_buffer);
                }
                line_index = 0;
                console_printf("debuger> ");
            } else if (c == 8 || c == 127) { // Backspace
                if (line_index > 0) {
                    line_index--;
                    console_printf("\b \b");
                }
            } else if (c >= 32 && c < 127 && line_index < (int)sizeof(line_buffer) - 1) {
                line_buffer[line_index++] = (char)c;
                char echo[2] = {(char)c, '\0'};
                console_print(echo);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================================
 * APPLICATION ENTRY POINT
 * ============================================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Debugger Application...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize Universal Console (USB Serial JTAG + UART0)
    console_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    console_printf("\r\n\r\n");
    console_printf("============================================================\r\n");
    console_printf("  AUTOMATED TEA BREWER - MOTOR & D8563TS RTC DEBUGGER       \r\n");
    console_printf("============================================================\r\n");

    // Initialize Stepper Motor Controller
    console_printf("[1/5] Initializing TMC2130 driver & gptimer...\r\n");
    ret = motor_init();
    if (ret == ESP_OK) {
        console_printf("      -> Motor subsystem: OK\r\n");
    } else {
        console_printf("      -> Motor subsystem: FAILED (%s)\r\n", esp_err_to_name(ret));
    }

    // Attempt loading saved calibration from NVS
    ret = motor_load_calibration();
    if (ret == ESP_OK) {
        console_printf("      -> Motor Calibration: LOADED\r\n");
    } else {
        console_printf("      -> Motor Calibration: NONE (Run 'cal' to calibrate)\r\n");
    }

    // Initialize D8563TS I2C RTC Module
    console_printf("[2/5] Initializing D8563TS I2C RTC (SDA: GPIO %d, SCL: GPIO %d)...\r\n",
                   PIN_I2C_SDA, PIN_I2C_SCL);
    ret = d8563_init(PIN_I2C_SDA, PIN_I2C_SCL, 50000);
    if (ret == ESP_OK) {
        if (d8563_is_connected()) {
            console_printf("      -> D8563TS RTC (0x51): ONLINE (OK)\r\n");
        } else {
            console_printf("      -> D8563TS RTC (0x51): NOT DETECTED (Run 'scan' to inspect bus)\r\n");
        }
    } else {
        console_printf("      -> I2C Bus Init: FAILED (%s)\r\n", esp_err_to_name(ret));
    }

    // Initialize VL53L0X Laser Distance Sensor
    console_printf("[3/5] Initializing VL53L0X Laser Distance Sensor (0x29)...\r\n");
    ret = distance_sensor_init(NULL);
    if (ret == ESP_OK) {
        console_printf("      -> VL53L0X ToF (0x29): ONLINE (OK)\r\n");
    } else {
        console_printf("      -> VL53L0X ToF (0x29): NOT DETECTED (Run 'scan' / 'dist' to test)\r\n");
    }

    // Initialize MLX90614 Infrared Temperature Sensor
    console_printf("[4/6] Initializing MLX90614 IR Thermometer (0x5A)...\r\n");
    ret = mlx90614_init(PIN_I2C_SDA, PIN_I2C_SCL);
    if (ret == ESP_OK) {
        console_printf("      -> MLX90614 IR (0x5A): ONLINE (OK)\r\n");
    } else {
        console_printf("      -> MLX90614 IR (0x5A): NOT DETECTED (Run 'scan' / 'temp' to test)\r\n");
    }

    // Initialize Induction Cooker Control Pin
    console_printf("[5/6] Initializing Induction Cooker Control (GPIO %d)...\r\n", PIN_INDUCTION);
    induction_init();
    console_printf("      -> Induction Cooker Pin (GPIO %d): READY (OFF)\r\n", PIN_INDUCTION);

    console_printf("[6/6] Starting Interactive CLI Console...\r\n\r\n");

    // Create interactive debugger console task
    xTaskCreate(console_task, "dbg_console", 4096, NULL, 5, NULL);
}
