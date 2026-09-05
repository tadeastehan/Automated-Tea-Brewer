/**
 * @file rtc.c
 * @brief D8563TS / PCF8563 Real-Time Clock Implementation
 * 
 * Uses shared bit-bang I2C engine compatible with all sensors on the bus.
 */

#include "rtc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "main_pins.h"
#include <string.h>

static const char *TAG = "RTC";

#define D8563_REG_CTRL1     0x00
#define D8563_REG_CTRL2     0x01
#define D8563_REG_VL_SEC    0x02
#define D8563_REG_MIN       0x03
#define D8563_REG_HOUR      0x04
#define D8563_REG_DAY       0x05
#define D8563_REG_WDAY      0x06
#define D8563_REG_CENT_MON  0x07
#define D8563_REG_YEAR      0x08

static bool s_initialized = false;

/* ============================================================================
   SHARED BIT-BANG I2C ENGINE
   ============================================================================ */
#define I2C_DELAY_US 10

static void bb_configure_pins(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << PIN_I2C_SDA) | (1ULL << PIN_I2C_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level(PIN_I2C_SDA, 1);
    gpio_set_level(PIN_I2C_SCL, 1);
}

static void bb_start(void)
{
    gpio_set_level(PIN_I2C_SDA, 1);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SDA, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(I2C_DELAY_US);
}

static void bb_stop(void)
{
    gpio_set_level(PIN_I2C_SDA, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SDA, 1);
    esp_rom_delay_us(I2C_DELAY_US);
}

static bool bb_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        int bit = (byte & 0x80) ? 1 : 0;
        gpio_set_level(PIN_I2C_SDA, bit);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(PIN_I2C_SCL, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(PIN_I2C_SCL, 0);
        esp_rom_delay_us(I2C_DELAY_US);
        byte <<= 1;
    }

    gpio_set_level(PIN_I2C_SDA, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    int ack_bit = gpio_get_level(PIN_I2C_SDA);
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(I2C_DELAY_US);

    return (ack_bit == 0);
}

static uint8_t bb_read_byte(bool send_ack)
{
    uint8_t byte = 0;
    gpio_set_level(PIN_I2C_SDA, 1);

    for (int i = 0; i < 8; i++) {
        gpio_set_level(PIN_I2C_SCL, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        byte = (byte << 1) | (gpio_get_level(PIN_I2C_SDA) ? 1 : 0);
        gpio_set_level(PIN_I2C_SCL, 0);
        esp_rom_delay_us(I2C_DELAY_US);
    }

    gpio_set_level(PIN_I2C_SDA, send_ack ? 0 : 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(PIN_I2C_SDA, 1);

    return byte;
}

static bool d8563_bb_write_regs(uint8_t reg, const uint8_t *data, size_t len)
{
    bb_configure_pins();
    bb_start();
    if (!bb_write_byte((D8563_I2C_ADDR << 1) | 0)) {
        bb_stop();
        return false;
    }
    if (!bb_write_byte(reg)) {
        bb_stop();
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (!bb_write_byte(data[i])) {
            bb_stop();
            return false;
        }
    }
    bb_stop();
    return true;
}

static bool d8563_bb_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    bb_configure_pins();
    bb_start();
    if (!bb_write_byte((D8563_I2C_ADDR << 1) | 0)) {
        bb_stop();
        return false;
    }
    if (!bb_write_byte(reg)) {
        bb_stop();
        return false;
    }
    bb_start(); // Repeated START
    if (!bb_write_byte((D8563_I2C_ADDR << 1) | 1)) {
        bb_stop();
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        bool ack = (i < len - 1);
        data[i] = bb_read_byte(ack);
    }
    bb_stop();
    return true;
}

/* ============================================================================
   BCD CONVERSIONS
   ============================================================================ */

static inline uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

static inline uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

/* ============================================================================
   PUBLIC FUNCTIONS
   ============================================================================ */

esp_err_t rtc_clock_init(i2c_master_bus_handle_t i2c_bus)
{
    (void)i2c_bus;
    ESP_LOGI(TAG, "Initializing D8563TS / PCF8563 RTC (0x51)...");

    bb_configure_pins();

    uint8_t test_byte = 0;
    if (!d8563_bb_read_regs(D8563_REG_CTRL1, &test_byte, 1)) {
        ESP_LOGE(TAG, "D8563TS RTC not responding at I2C address 0x51");
        s_initialized = false;
        return ESP_ERR_NOT_FOUND;
    }

    // Ensure STOP bit (bit 5 of CTRL1) is 0 so the 32.768kHz oscillator runs
    uint8_t ctrl1 = 0x00;
    d8563_bb_write_regs(D8563_REG_CTRL1, &ctrl1, 1);

    s_initialized = true;
    ESP_LOGI(TAG, "D8563TS RTC initialized successfully (0x51)");
    return ESP_OK;
}

esp_err_t rtc_get_time(struct tm *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[7];
    if (!d8563_bb_read_regs(D8563_REG_VL_SEC, raw, 7)) {
        return ESP_FAIL;
    }

    memset(time, 0, sizeof(struct tm));

    // 0x02: Seconds (bit 7 = VL flag)
    time->tm_sec = bcd2dec(raw[0] & 0x7F);

    // 0x03: Minutes (bit 7 = unused)
    time->tm_min = bcd2dec(raw[1] & 0x7F);

    // 0x04: Hours (bit 6-7 = unused)
    time->tm_hour = bcd2dec(raw[2] & 0x3F);

    // 0x05: Days (bit 6-7 = unused)
    time->tm_mday = bcd2dec(raw[3] & 0x3F);

    // 0x06: Weekday (0-6)
    time->tm_wday = raw[4] & 0x07;

    // 0x07: Century + Month (bit 7 = C, bit 0-4 = Month)
    time->tm_mon = bcd2dec(raw[5] & 0x1F) - 1; // tm_mon is 0-11

    // 0x08: Year (00-99 -> 2000-2099)
    uint16_t year = 2000 + bcd2dec(raw[6]);
    time->tm_year = year - 1900; // tm_year is years since 1900

    time->tm_isdst = -1;

    return ESP_OK;
}

esp_err_t rtc_set_time(const struct tm *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Stop clock during setup
    uint8_t ctrl1_stop = 0x20;
    d8563_bb_write_regs(D8563_REG_CTRL1, &ctrl1_stop, 1);

    uint8_t raw[7];
    raw[0] = dec2bcd(time->tm_sec) & 0x7F;             // VL = 0
    raw[1] = dec2bcd(time->tm_min) & 0x7F;
    raw[2] = dec2bcd(time->tm_hour) & 0x3F;
    raw[3] = dec2bcd(time->tm_mday) & 0x3F;
    raw[4] = (uint8_t)(time->tm_wday & 0x07);
    raw[5] = dec2bcd((time->tm_mon + 1)) & 0x1F;       // Century bit = 0 (2000s)
    raw[6] = dec2bcd((time->tm_year + 1900) % 100);

    if (!d8563_bb_write_regs(D8563_REG_VL_SEC, raw, 7)) {
        return ESP_FAIL;
    }

    // 2. Start clock
    uint8_t ctrl1_run = 0x00;
    d8563_bb_write_regs(D8563_REG_CTRL1, &ctrl1_run, 1);

    return ESP_OK;
}

esp_err_t rtc_get_time_with_timezone(struct tm *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = rtc_get_time(time);
    if (ret != ESP_OK) {
        return ret;
    }

    time_t raw_time = mktime(time);
    if (raw_time == (time_t)-1) {
        return ESP_FAIL;
    }

    // Add GMT+1 offset (3600 seconds)
    raw_time += 3600;

    struct tm *adjusted = localtime(&raw_time);
    if (adjusted == NULL) {
        return ESP_FAIL;
    }

    *time = *adjusted;
    return ESP_OK;
}

bool rtc_is_initialized(void)
{
    return s_initialized;
}
