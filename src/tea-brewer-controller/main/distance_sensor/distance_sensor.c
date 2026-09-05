/**
 * @file distance_sensor.c
 * @brief VL53L0X Time-of-Flight Distance Sensor Driver Implementation
 * 
 * Rewritten for the new ESP-IDF I2C master API.
 */

#include "distance_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main_pins.h"
#include <string.h>

static const char *TAG = "DISTANCE_SENSOR";

/* ============================================
   VL53L0X Configuration
   ============================================ */
#define VL53L0X_I2C_ADDR        0x29    /**< Default I2C address */
#define VL53L0X_I2C_FREQ_HZ     400000  /**< I2C frequency (400kHz) */
#define VL53L0X_TIMEOUT_MS      500     /**< Default timeout */

/* VL53L0X Register Addresses */
#define SYSRANGE_START                          0x00
#define SYSTEM_THRESH_HIGH                      0x0C
#define SYSTEM_THRESH_LOW                       0x0E
#define SYSTEM_SEQUENCE_CONFIG                  0x01
#define SYSTEM_RANGE_CONFIG                     0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define SYSTEM_INTERRUPT_CLEAR                  0x0B
#define RESULT_INTERRUPT_STATUS                 0x13
#define RESULT_RANGE_STATUS                     0x14
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN   0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN    0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF   0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF    0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF             0xB6
#define ALGO_PART_TO_PART_RANGE_OFFSET_MM       0x28
#define I2C_SLAVE_DEVICE_ADDRESS                0x8A
#define MSRC_CONFIG_CONTROL                     0x60
#define PRE_RANGE_CONFIG_MIN_SNR                0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT      0x64
#define FINAL_RANGE_CONFIG_MIN_SNR              0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI        0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO        0x62
#define PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO      0x52
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO    0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS   0x20
#define MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define SOFT_RESET_GO2_SOFT_RESET_N             0xBF
#define IDENTIFICATION_MODEL_ID                 0xC0
#define IDENTIFICATION_REVISION_ID              0xC2
#define OSC_CALIBRATE_VAL                       0xF8
#define GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD     0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET        0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE        0x80
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define ALGO_PHASECAL_LIM                       0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

#include "esp_rom_sys.h"
#include "driver/gpio.h"

/* ============================================
   Static Variables
   ============================================ */
static bool s_initialized = false;
static uint8_t s_stop_variable = 0;
static uint32_t s_measurement_timing_budget_us = 0;

/* ============================================
   Universal Bit-Bang I2C Engine (Bus Sharing)
   ============================================ */
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

static bool bb_vl53l0x_write(uint8_t reg, const uint8_t *data, size_t len)
{
    bb_configure_pins();
    bb_start();
    if (!bb_write_byte(0x52)) { bb_stop(); return false; } // 0x29 << 1 | 0
    if (!bb_write_byte(reg)) { bb_stop(); return false; }
    for (size_t i = 0; i < len; i++) {
        if (!bb_write_byte(data[i])) { bb_stop(); return false; }
    }
    bb_stop();
    return true;
}

static bool bb_vl53l0x_read(uint8_t reg, uint8_t *data, size_t len)
{
    bb_configure_pins();
    bb_start();
    if (!bb_write_byte(0x52)) { bb_stop(); return false; } // 0x29 << 1 | 0
    if (!bb_write_byte(reg)) { bb_stop(); return false; }
    bb_start(); // Repeated START
    if (!bb_write_byte(0x53)) { bb_stop(); return false; } // 0x29 << 1 | 1
    for (size_t i = 0; i < len; i++) {
        bool ack = (i < len - 1);
        data[i] = bb_read_byte(ack);
    }
    bb_stop();
    return true;
}

/* ============================================
   I2C Helper Functions
   ============================================ */

static esp_err_t vl53l0x_write_reg8(uint8_t reg, uint8_t value)
{
    return bb_vl53l0x_write(reg, &value, 1) ? ESP_OK : ESP_FAIL;
}

static esp_err_t vl53l0x_write_reg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return bb_vl53l0x_write(reg, buf, 2) ? ESP_OK : ESP_FAIL;
}

static esp_err_t vl53l0x_write_reg32(uint8_t reg, uint32_t value)
{
    uint8_t buf[4] = { 
        (uint8_t)(value >> 24), 
        (uint8_t)(value >> 16), 
        (uint8_t)(value >> 8), 
        (uint8_t)(value & 0xFF) 
    };
    return bb_vl53l0x_write(reg, buf, 4) ? ESP_OK : ESP_FAIL;
}

static esp_err_t vl53l0x_write_multi(uint8_t reg, const uint8_t *src, uint8_t count)
{
    return bb_vl53l0x_write(reg, src, count) ? ESP_OK : ESP_FAIL;
}

static esp_err_t vl53l0x_read_reg8(uint8_t reg, uint8_t *value)
{
    return bb_vl53l0x_read(reg, value, 1) ? ESP_OK : ESP_FAIL;
}

static esp_err_t vl53l0x_read_reg16(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    if (!bb_vl53l0x_read(reg, buf, 2)) return ESP_FAIL;
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

static esp_err_t vl53l0x_read_multi(uint8_t reg, uint8_t *dst, uint8_t count)
{
    return bb_vl53l0x_read(reg, dst, count) ? ESP_OK : ESP_FAIL;
}

/* ============================================
   VL53L0X Internal Functions
   ============================================ */

/* Decode VCSEL period from register value */
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

/* Encode VCSEL period to register value */
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

/* Calculate macro period in nanoseconds */
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

static uint16_t decode_timeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint16_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

typedef struct {
    uint8_t tcc;
    uint8_t msrc;
    uint8_t dss;
    uint8_t pre_range;
    uint8_t final_range;
} sequence_step_enables_t;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} sequence_step_timeouts_t;

static esp_err_t get_sequence_step_enables(sequence_step_enables_t *enables)
{
    uint8_t sequence_config;
    esp_err_t ret = vl53l0x_read_reg8(SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    if (ret != ESP_OK) return ret;

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;

    return ESP_OK;
}

static esp_err_t get_vcsel_pulse_period(uint8_t type, uint8_t *period)
{
    uint8_t reg_val;
    esp_err_t ret;
    
    if (type == 0) { /* Pre-range */
        ret = vl53l0x_read_reg8(PRE_RANGE_CONFIG_VCSEL_PERIOD, &reg_val);
    } else { /* Final range */
        ret = vl53l0x_read_reg8(FINAL_RANGE_CONFIG_VCSEL_PERIOD, &reg_val);
    }
    
    if (ret == ESP_OK) {
        *period = decodeVcselPeriod(reg_val);
    }
    return ret;
}

static esp_err_t get_sequence_step_timeouts(sequence_step_enables_t *enables, sequence_step_timeouts_t *timeouts)
{
    esp_err_t ret;
    uint8_t reg_val;
    uint16_t reg_val16;

    ret = get_vcsel_pulse_period(0, &reg_val);
    if (ret != ESP_OK) return ret;
    timeouts->pre_range_vcsel_period_pclks = reg_val;

    ret = vl53l0x_read_reg8(MSRC_CONFIG_TIMEOUT_MACROP, &reg_val);
    if (ret != ESP_OK) return ret;
    timeouts->msrc_dss_tcc_mclks = reg_val + 1;
    timeouts->msrc_dss_tcc_us = timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks, 
                                                     timeouts->pre_range_vcsel_period_pclks);

    ret = vl53l0x_read_reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg_val16);
    if (ret != ESP_OK) return ret;
    timeouts->pre_range_mclks = decode_timeout(reg_val16);
    timeouts->pre_range_us = timeout_mclks_to_us(timeouts->pre_range_mclks,
                                                  timeouts->pre_range_vcsel_period_pclks);

    ret = get_vcsel_pulse_period(1, &reg_val);
    if (ret != ESP_OK) return ret;
    timeouts->final_range_vcsel_period_pclks = reg_val;

    ret = vl53l0x_read_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg_val16);
    if (ret != ESP_OK) return ret;
    timeouts->final_range_mclks = decode_timeout(reg_val16);

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeout_mclks_to_us(timeouts->final_range_mclks,
                                                    timeouts->final_range_vcsel_period_pclks);

    return ESP_OK;
}

static esp_err_t get_measurement_timing_budget(uint32_t *budget_us)
{
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;
    esp_err_t ret;

    const uint16_t StartOverhead = 1910;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    *budget_us = StartOverhead + EndOverhead;

    ret = get_sequence_step_enables(&enables);
    if (ret != ESP_OK) return ret;

    ret = get_sequence_step_timeouts(&enables, &timeouts);
    if (ret != ESP_OK) return ret;

    if (enables.tcc) {
        *budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        *budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        *budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        *budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        *budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    s_measurement_timing_budget_us = *budget_us;
    return ESP_OK;
}

static esp_err_t set_measurement_timing_budget(uint32_t budget_us)
{
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;
    esp_err_t ret;

    const uint16_t StartOverhead = 1320;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    ret = get_sequence_step_enables(&enables);
    if (ret != ESP_OK) return ret;

    ret = get_sequence_step_timeouts(&enables, &timeouts);
    if (ret != ESP_OK) return ret;

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us) {
            return ESP_ERR_INVALID_ARG;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint32_t final_range_timeout_mclks = timeout_us_to_mclks(final_range_timeout_us,
                                                                  timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        ret = vl53l0x_write_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                   encode_timeout(final_range_timeout_mclks));
        if (ret != ESP_OK) return ret;

        s_measurement_timing_budget_us = budget_us;
    }

    return ESP_OK;
}

static esp_err_t perform_single_ref_calibration(uint8_t vhv_init_byte)
{
    esp_err_t ret;
    uint8_t reg_val;
    int64_t start_time = esp_timer_get_time();

    ret = vl53l0x_write_reg8(SYSRANGE_START, 0x01 | vhv_init_byte);
    if (ret != ESP_OK) return ret;

    while (1) {
        ret = vl53l0x_read_reg8(RESULT_INTERRUPT_STATUS, &reg_val);
        if (ret != ESP_OK) return ret;
        
        if ((reg_val & 0x07) != 0) break;
        
        if ((esp_timer_get_time() - start_time) > (VL53L0X_TIMEOUT_MS * 1000)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ret = vl53l0x_write_reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(SYSRANGE_START, 0x00);
    return ret;
}

static esp_err_t get_spad_info(uint8_t *count, uint8_t *type_is_aperture)
{
    esp_err_t ret;
    uint8_t tmp;
    int64_t start_time;

    ret = vl53l0x_write_reg8(0x80, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x00);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0xFF, 0x06);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_read_reg8(0x83, &tmp);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x83, tmp | 0x04);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_write_reg8(0xFF, 0x07);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x81, 0x01);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0x80, 0x01);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0x94, 0x6b);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x83, 0x00);
    if (ret != ESP_OK) return ret;

    start_time = esp_timer_get_time();
    while (1) {
        ret = vl53l0x_read_reg8(0x83, &tmp);
        if (ret != ESP_OK) return ret;
        if (tmp != 0x00) break;
        if ((esp_timer_get_time() - start_time) > (VL53L0X_TIMEOUT_MS * 1000)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ret = vl53l0x_write_reg8(0x83, 0x01);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_read_reg8(0x92, &tmp);
    if (ret != ESP_OK) return ret;

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    ret = vl53l0x_write_reg8(0x81, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x06);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_read_reg8(0x83, &tmp);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x83, tmp & ~0x04);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x01);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0xFF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x80, 0x00);

    return ret;
}

static esp_err_t set_signal_rate_limit(float limit_mcps)
{
    if (limit_mcps < 0 || limit_mcps > 511.99f) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Q9.7 fixed point format */
    return vl53l0x_write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 
                                (uint16_t)(limit_mcps * (1 << 7)));
}

static esp_err_t vl53l0x_data_init(void)
{
    esp_err_t ret;
    uint8_t reg_val;

    /* Set 2.8V mode */
    ret = vl53l0x_read_reg8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &reg_val);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, reg_val | 0x01);
    if (ret != ESP_OK) return ret;

    /* Set I2C standard mode */
    ret = vl53l0x_write_reg8(0x88, 0x00);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0x80, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_read_reg8(0x91, &s_stop_variable);
    if (ret != ESP_OK) return ret;
    
    ret = vl53l0x_write_reg8(0x00, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x80, 0x00);
    if (ret != ESP_OK) return ret;

    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    ret = vl53l0x_read_reg8(MSRC_CONFIG_CONTROL, &reg_val);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(MSRC_CONFIG_CONTROL, reg_val | 0x12);
    if (ret != ESP_OK) return ret;

    /* Set signal rate limit to 0.25 MCPS */
    ret = set_signal_rate_limit(0.25f);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    return ret;
}

static esp_err_t vl53l0x_static_init(void)
{
    esp_err_t ret;
    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    uint8_t ref_spad_map[6];

    ret = get_spad_info(&spad_count, &spad_type_is_aperture);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_read_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    if (ret != ESP_OK) return ret;

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    ret = vl53l0x_write_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    if (ret != ESP_OK) return ret;

    /* Load tuning settings */
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x09, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x10, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x11, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x24, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x25, 0xFF); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x75, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x4E, 0x2C); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x48, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x30, 0x20); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x30, 0x09); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x54, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x31, 0x04); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x32, 0x03); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x40, 0x83); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x46, 0x25); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x60, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x27, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x50, 0x06); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x51, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x52, 0x96); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x56, 0x08); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x57, 0x30); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x61, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x62, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x64, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x65, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x66, 0xA0); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x22, 0x32); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x47, 0x14); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x49, 0xFF); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x4A, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x7A, 0x0A); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x7B, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x78, 0x21); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x23, 0x34); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x42, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x44, 0xFF); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x45, 0x26); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x46, 0x05); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x40, 0x40); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x0E, 0x06); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x20, 0x1A); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x43, 0x40); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x34, 0x03); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x35, 0x44); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x31, 0x04); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x4B, 0x09); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x4C, 0x05); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x4D, 0x04); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x44, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x45, 0x20); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x47, 0x08); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x48, 0x28); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x67, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x70, 0x04); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x71, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x72, 0xFE); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x76, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x77, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x0D, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x80, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x01, 0xF8); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x8E, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x01); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00); if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x80, 0x00); if (ret != ESP_OK) return ret;

    /* Set interrupt config */
    ret = vl53l0x_write_reg8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    if (ret != ESP_OK) return ret;

    uint8_t reg_val;
    ret = vl53l0x_read_reg8(GPIO_HV_MUX_ACTIVE_HIGH, &reg_val);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(GPIO_HV_MUX_ACTIVE_HIGH, reg_val & ~0x10);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (ret != ESP_OK) return ret;

    uint32_t budget_us;
    ret = get_measurement_timing_budget(&budget_us);
    if (ret != ESP_OK) return ret;

    /* Disable MSRC and TCC by default */
    ret = vl53l0x_write_reg8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    if (ret != ESP_OK) return ret;

    /* Recalculate timing budget */
    ret = set_measurement_timing_budget(s_measurement_timing_budget_us);

    return ret;
}

static esp_err_t vl53l0x_perform_ref_calibration(void)
{
    esp_err_t ret;

    /* VHV calibration */
    ret = vl53l0x_write_reg8(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;
    ret = perform_single_ref_calibration(0x40);
    if (ret != ESP_OK) return ret;

    /* Phase calibration */
    ret = vl53l0x_write_reg8(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (ret != ESP_OK) return ret;
    ret = perform_single_ref_calibration(0x00);
    if (ret != ESP_OK) return ret;

    /* Restore sequence config */
    ret = vl53l0x_write_reg8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return ret;
}

/* ============================================
   Public Functions
   ============================================ */

esp_err_t distance_sensor_init(i2c_master_bus_handle_t bus_handle)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing VL53L0X distance sensor...");
    bb_configure_pins();

    /* Check device ID */
    uint8_t model_id = 0;
    esp_err_t ret = vl53l0x_read_reg8(IDENTIFICATION_MODEL_ID, &model_id);
    if (ret != ESP_OK || model_id != 0xEE) {
        ESP_LOGE(TAG, "VL53L0X not found (ID: 0x%02X, expected 0xEE)", model_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "VL53L0X found (ID: 0x%02X)", model_id);

    /* Initialize device */
    ret = vl53l0x_data_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Data init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = vl53l0x_static_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Static init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = vl53l0x_perform_ref_calibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ref calibration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "VL53L0X initialized successfully");
    return ESP_OK;
}

esp_err_t distance_sensor_deinit(void)
{
    s_initialized = false;
    ESP_LOGI(TAG, "VL53L0X deinitialized");
    return ESP_OK;
}

bool distance_sensor_is_initialized(void)
{
    return s_initialized;
}

i2c_master_bus_handle_t distance_sensor_get_bus_handle(void)
{
    return NULL;
}

esp_err_t distance_sensor_get_distance(uint16_t *distance_mm)
{
    if (distance_mm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    uint8_t reg_val;
    int64_t start_time;

    /* Start single measurement */
    ret = vl53l0x_write_reg8(0x80, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x91, s_stop_variable);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x80, 0x00);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(SYSRANGE_START, 0x01);
    if (ret != ESP_OK) return ret;

    /* Wait for start */
    start_time = esp_timer_get_time();
    while (1) {
        ret = vl53l0x_read_reg8(SYSRANGE_START, &reg_val);
        if (ret != ESP_OK) return ret;
        if ((reg_val & 0x01) == 0) break;
        if ((esp_timer_get_time() - start_time) > (VL53L0X_TIMEOUT_MS * 1000)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Wait for measurement complete */
    start_time = esp_timer_get_time();
    while (1) {
        ret = vl53l0x_read_reg8(RESULT_INTERRUPT_STATUS, &reg_val);
        if (ret != ESP_OK) return ret;
        if ((reg_val & 0x07) != 0) break;
        if ((esp_timer_get_time() - start_time) > (VL53L0X_TIMEOUT_MS * 1000)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Read result */
    ret = vl53l0x_read_reg16(RESULT_RANGE_STATUS + 10, distance_mm);
    if (ret != ESP_OK) return ret;

    /* Clear interrupt */
    ret = vl53l0x_write_reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return ret;
}

esp_err_t distance_sensor_start_continuous(uint32_t period_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    ret = vl53l0x_write_reg8(0x80, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x91, s_stop_variable);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x80, 0x00);
    if (ret != ESP_OK) return ret;

    if (period_ms != 0) {
        /* Timed mode */
        uint16_t osc_calibrate_val;
        ret = vl53l0x_read_reg16(OSC_CALIBRATE_VAL, &osc_calibrate_val);
        if (ret != ESP_OK) return ret;

        if (osc_calibrate_val != 0) {
            period_ms *= osc_calibrate_val;
        }

        ret = vl53l0x_write_reg32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        if (ret != ESP_OK) return ret;

        ret = vl53l0x_write_reg8(SYSRANGE_START, 0x04);
    } else {
        /* Back-to-back mode */
        ret = vl53l0x_write_reg8(SYSRANGE_START, 0x02);
    }

    return ret;
}

esp_err_t distance_sensor_stop_continuous(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    ret = vl53l0x_write_reg8(SYSRANGE_START, 0x01);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg8(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x91, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0x00, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg8(0xFF, 0x00);

    return ret;
}

esp_err_t distance_sensor_read_continuous(uint16_t *distance_mm)
{
    if (distance_mm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    uint8_t reg_val;
    int64_t start_time = esp_timer_get_time();

    /* Wait for measurement complete */
    while (1) {
        ret = vl53l0x_read_reg8(RESULT_INTERRUPT_STATUS, &reg_val);
        if (ret != ESP_OK) return ret;
        if ((reg_val & 0x07) != 0) break;
        if ((esp_timer_get_time() - start_time) > (VL53L0X_TIMEOUT_MS * 1000)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Read result */
    ret = vl53l0x_read_reg16(RESULT_RANGE_STATUS + 10, distance_mm);
    if (ret != ESP_OK) return ret;

    /* Clear interrupt */
    ret = vl53l0x_write_reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return ret;
}
