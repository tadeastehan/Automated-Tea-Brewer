/**
 * @file d8563.c
 * @brief D8563TS / PCF8563 Real-Time Clock Driver Implementation
 * 
 * Uses robust software bit-banging with proper START, STOP, ACK/NACK signaling
 * to ensure 100% compatibility across all D8563TS / PCF8563 silicon revisions.
 */

#include "d8563.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "D8563";

static bool initialized = false;
static gpio_num_t cur_sda_pin = GPIO_NUM_22;
static gpio_num_t cur_scl_pin = GPIO_NUM_23;
static uint32_t cur_speed_hz = 50000;

#define I2C_DELAY_US 10

/* Helper: BCD to Decimal */
static inline uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

/* Helper: Decimal to BCD */
static inline uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

const char *d8563_weekday_name(int wday)
{
    static const char *names[] = {
        "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
    };
    if (wday >= 0 && wday < 7) {
        return names[wday];
    }
    return "Unknown";
}

void d8563_get_pins(gpio_num_t *sda, gpio_num_t *scl, uint32_t *speed_hz)
{
    if (sda) *sda = cur_sda_pin;
    if (scl) *scl = cur_scl_pin;
    if (speed_hz) *speed_hz = cur_speed_hz;
}

/* ============================================================================
   LOW-LEVEL BIT-BANG SIGNALS
   ============================================================================ */

static void bb_configure_pins(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << cur_sda_pin) | (1ULL << cur_scl_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level(cur_sda_pin, 1);
    gpio_set_level(cur_scl_pin, 1);
}

void d8563_recover_bus(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    bb_configure_pins();

    gpio_set_level(sda_pin, 1);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);

    // 9 clock pulses on SCL
    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl_pin, 0);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(scl_pin, 1);
        esp_rom_delay_us(I2C_DELAY_US);
    }

    // Generate clean STOP
    gpio_set_level(sda_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(sda_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
}

static void bb_start(void)
{
    gpio_set_level(cur_sda_pin, 1);
    gpio_set_level(cur_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_sda_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_scl_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
}

static void bb_stop(void)
{
    gpio_set_level(cur_sda_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_sda_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
}

static bool bb_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        int bit = (byte & 0x80) ? 1 : 0;
        gpio_set_level(cur_sda_pin, bit);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(cur_scl_pin, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        gpio_set_level(cur_scl_pin, 0);
        esp_rom_delay_us(I2C_DELAY_US);
        byte <<= 1;
    }

    // Read ACK on 9th clock
    gpio_set_level(cur_sda_pin, 1); // Release SDA to let slave pull it low
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    int ack_bit = gpio_get_level(cur_sda_pin);
    gpio_set_level(cur_scl_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);

    return (ack_bit == 0); // ACK = LOW (0)
}

static uint8_t bb_read_byte(bool send_ack)
{
    uint8_t byte = 0;
    gpio_set_level(cur_sda_pin, 1); // Release SDA for reading

    for (int i = 0; i < 8; i++) {
        gpio_set_level(cur_scl_pin, 1);
        esp_rom_delay_us(I2C_DELAY_US);
        byte = (byte << 1) | (gpio_get_level(cur_sda_pin) ? 1 : 0);
        gpio_set_level(cur_scl_pin, 0);
        esp_rom_delay_us(I2C_DELAY_US);
    }

    // Send ACK (0) or NACK (1) on 9th clock
    gpio_set_level(cur_sda_pin, send_ack ? 0 : 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_scl_pin, 1);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_scl_pin, 0);
    esp_rom_delay_us(I2C_DELAY_US);
    gpio_set_level(cur_sda_pin, 1); // Release SDA

    return byte;
}

static bool bb_read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    bb_start();
    // Step 1: Write device address 0xA2 (Write mode)
    if (!bb_write_byte(0xA2)) {
        bb_stop();
        return false;
    }
    // Step 2: Write starting register pointer
    if (!bb_write_byte(start_reg)) {
        bb_stop();
        return false;
    }
    // Step 3: Repeated START
    bb_start();
    // Step 4: Write device address 0xA3 (Read mode)
    if (!bb_write_byte(0xA3)) {
        bb_stop();
        return false;
    }
    // Step 5: Read data bytes (ACK all except the last byte, which gets NACK)
    for (size_t i = 0; i < len; i++) {
        bool ack = (i < len - 1);
        data[i] = bb_read_byte(ack);
    }
    bb_stop();
    return true;
}

static bool bb_write_regs(uint8_t start_reg, const uint8_t *data, size_t len)
{
    bb_start();
    if (!bb_write_byte(0xA2)) {
        bb_stop();
        return false;
    }
    if (!bb_write_byte(start_reg)) {
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

/* ============================================================================
   PUBLIC DRIVER API
   ============================================================================ */

esp_err_t d8563_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t speed_hz)
{
    cur_sda_pin = sda_pin;
    cur_scl_pin = scl_pin;
    if (speed_hz > 0) cur_speed_hz = speed_hz;

    bb_configure_pins();
    d8563_recover_bus(sda_pin, scl_pin);

    initialized = true;
    ESP_LOGI(TAG, "D8563 initialized (SDA=GPIO %d, SCL=GPIO %d)", sda_pin, scl_pin);
    return ESP_OK;
}

esp_err_t d8563_reinit_pins(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t speed_hz)
{
    return d8563_init(sda_pin, scl_pin, speed_hz);
}

bool d8563_is_connected(void)
{
    uint8_t dummy = 0;
    return bb_read_regs(0x00, &dummy, 1);
}

esp_err_t d8563_get_time(d8563_time_t *time)
{
    if (time == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t data[7];
    if (!bb_read_regs(D8563_REG_VL_SECONDS, data, 7)) {
        return ESP_FAIL;
    }

    time->voltage_low = (data[0] & 0x80) != 0;
    time->second      = bcd2dec(data[0] & 0x7F);
    time->minute      = bcd2dec(data[1] & 0x7F);
    time->hour        = bcd2dec(data[2] & 0x3F);
    time->day         = bcd2dec(data[3] & 0x3F);
    time->weekday     = data[4] & 0x07;
    bool century      = (data[5] & 0x80) != 0;
    time->month       = bcd2dec(data[5] & 0x1F);
    int year_2digits  = bcd2dec(data[6]);
    time->year        = (century ? 1900 : 2000) + year_2digits;

    return ESP_OK;
}

esp_err_t d8563_set_time(const d8563_time_t *time)
{
    if (time == NULL) return ESP_ERR_INVALID_ARG;

    // Step 1: Set STOP bit in CTRL1 so registers don't increment while writing
    uint8_t ctrl_stop = 0x20;
    if (!bb_write_regs(D8563_REG_CTRL1, &ctrl_stop, 1)) {
        return ESP_FAIL;
    }

    // Step 2: Write Time/Date registers starting at 0x02
    uint8_t buffer[7];
    buffer[0] = dec2bcd(time->second) & 0x7F; // Bit 7 = 0 (Clears VL Voltage Low bit)
    buffer[1] = dec2bcd(time->minute) & 0x7F;
    buffer[2] = dec2bcd(time->hour) & 0x3F;
    buffer[3] = dec2bcd(time->day) & 0x3F;
    buffer[4] = (uint8_t)(time->weekday & 0x07);
    uint8_t century_bit = (time->year < 2000) ? 0x80 : 0x00;
    buffer[5] = (dec2bcd(time->month) & 0x1F) | century_bit;
    buffer[6] = dec2bcd(time->year % 100);

    if (!bb_write_regs(D8563_REG_VL_SECONDS, buffer, 7)) {
        return ESP_FAIL;
    }

    // Step 3: Clear STOP bit in CTRL1 so RTC oscillator and counting start
    uint8_t ctrl_start = 0x00;
    if (!bb_write_regs(D8563_REG_CTRL1, &ctrl_start, 1)) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t d8563_get_status(d8563_status_t *status)
{
    if (status == NULL) return ESP_ERR_INVALID_ARG;

    memset(status, 0, sizeof(d8563_status_t));
    if (!bb_read_regs(0x00, status->raw_regs, 16)) {
        status->is_present = false;
        return ESP_FAIL;
    }

    status->is_present = true;
    status->is_running = (status->raw_regs[0] & 0x20) == 0;
    status->voltage_low = (status->raw_regs[2] & 0x80) != 0;

    d8563_get_time(&status->time);
    return ESP_OK;
}

void d8563_diagnose_bus(void)
{
    extern void console_printf(const char *fmt, ...);

    int sda_lvl = gpio_get_level(cur_sda_pin);
    int scl_lvl = gpio_get_level(cur_scl_pin);

    console_printf("--- I2C Hardware Bus Diagnostics ---\r\n");
    console_printf("  Configured Pins : SDA = GPIO %d, SCL = GPIO %d\r\n", cur_sda_pin, cur_scl_pin);
    console_printf("  SDA Pin Level   : %s (%d)\r\n", sda_lvl ? "HIGH (3.3V)" : "LOW (0V / Stuck)", sda_lvl);
    console_printf("  SCL Pin Level   : %s (%d)\r\n", scl_lvl ? "HIGH (3.3V)" : "LOW (0V / Stuck)", scl_lvl);

    if (sda_lvl == 0 || scl_lvl == 0) {
        console_printf("\r\n  [!] CAUTION: I2C line is held LOW!\r\n");
    } else {
        console_printf("  Pin Levels State: OK (Both idle HIGH at 3.3V)\r\n");
    }
    console_printf("------------------------------------\r\n\r\n");
}

void d8563_bitbang_ping(uint8_t addr_7bit)
{
    extern void console_printf(const char *fmt, ...);

    bb_configure_pins();
    uint8_t byte_wr = (addr_7bit << 1) | 0;
    console_printf("\r\n--- Bit-Bang I2C Ping (Address: 0x%02X, Byte: 0x%02X) ---\r\n", addr_7bit, byte_wr);

    bb_start();
    bool ack = bb_write_byte(byte_wr);
    bb_stop();

    if (ack) {
        console_printf("  Result: [ACK RECEIVED!] Slave at 0x%02X responded!\r\n", addr_7bit);
    } else {
        console_printf("  Result: [NACK - NO RESPONSE] Line remained HIGH at 9th clock.\r\n");
    }
    console_printf("----------------------------------------------------------\r\n\r\n");
}

void d8563_bitbang_scan(void)
{
    extern void console_printf(const char *fmt, ...);

    bb_configure_pins();
    d8563_recover_bus(cur_sda_pin, cur_scl_pin);

    console_printf("\r\n================ I2C BUS SCANNER (SDA: GPIO %d, SCL: GPIO %d) ================\r\n",
                   cur_sda_pin, cur_scl_pin);
    console_printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");

    int devices_found = 0;

    for (int i = 0; i < 128; i += 16) {
        console_printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            if (addr < 0x08 || addr > 0x77) {
                console_printf("   ");
            } else {
                bb_start();
                bool ack = bb_write_byte((addr << 1) | 0);
                bb_stop();

                if (ack) {
                    console_printf("%02x ", addr);
                    devices_found++;
                } else {
                    console_printf("-- ");
                }
            }
        }
        console_printf("\r\n");
    }

    console_printf("----------------------------------------------------------------------------------\r\n");
    console_printf("Devices found: %d\r\n", devices_found);

    if (devices_found > 0) {
        console_printf("  [+] Found I2C devices successfully!\r\n");
    }
    console_printf("==================================================================================\r\n\r\n");
}

void d8563_scan_i2c_bus(void)
{
    d8563_bitbang_scan();
}
