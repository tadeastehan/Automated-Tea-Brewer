/**
 * @file tmc2130.c
 * @brief TMC2130 Stepper Motor Driver implementation for ESP-IDF
 * 
 * Ported from Arduino TMC2130Stepper library by teemuatlut
 * Original: https://github.com/teemuatlut/TMC2130Stepper
 */

#include "tmc2130.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMC2130";

// Bit positions for GCONF register
#define GCONF_I_SCALE_ANALOG_POS        0
#define GCONF_INTERNAL_RSENSE_POS       1
#define GCONF_EN_PWM_MODE_POS           2
#define GCONF_ENC_COMMUTATION_POS       3
#define GCONF_SHAFT_POS                 4
#define GCONF_DIAG0_ERROR_POS           5
#define GCONF_DIAG0_OTPW_POS            6
#define GCONF_DIAG0_STALL_POS           7
#define GCONF_DIAG1_STALL_POS           8
#define GCONF_DIAG1_INDEX_POS           9
#define GCONF_DIAG1_ONSTATE_POS         10
#define GCONF_DIAG1_STEPS_SKIPPED_POS   11
#define GCONF_DIAG0_INT_PUSHPULL_POS    12
#define GCONF_DIAG1_PUSHPULL_POS        13
#define GCONF_SMALL_HYSTERESIS_POS      14
#define GCONF_STOP_ENABLE_POS           15
#define GCONF_DIRECT_MODE_POS           16

// Bit positions for CHOPCONF register
#define CHOPCONF_TOFF_POS               0
#define CHOPCONF_HSTRT_POS              4
#define CHOPCONF_HEND_POS               7
#define CHOPCONF_TBL_POS                15
#define CHOPCONF_VSENSE_POS             17
#define CHOPCONF_MRES_POS               24
#define CHOPCONF_INTPOL_POS             28
#define CHOPCONF_DEDGE_POS              29
#define CHOPCONF_DISS2G_POS             30
#define CHOPCONF_CHM_POS                14

// Bit masks
#define CHOPCONF_TOFF_MASK              0x0000000F
#define CHOPCONF_HSTRT_MASK             0x00000070
#define CHOPCONF_HEND_MASK              0x00000780
#define CHOPCONF_TBL_MASK               0x00018000
#define CHOPCONF_MRES_MASK              0x0F000000

// COOLCONF bit positions
#define COOLCONF_SEMIN_POS              0
#define COOLCONF_SEUP_POS               5
#define COOLCONF_SEMAX_POS              8
#define COOLCONF_SEDN_POS               13
#define COOLCONF_SEIMIN_POS             15
#define COOLCONF_SGT_POS                16
#define COOLCONF_SFILT_POS              24

// PWMCONF bit positions
#define PWMCONF_PWM_AMPL_POS            0
#define PWMCONF_PWM_GRAD_POS            8
#define PWMCONF_PWM_FREQ_POS            16
#define PWMCONF_PWM_AUTOSCALE_POS       18
#define PWMCONF_PWM_SYMMETRIC_POS       19
#define PWMCONF_FREEWHEEL_POS           20

// Default values
#define DEFAULT_F_CLK                   12000000  // 12 MHz internal clock

esp_err_t tmc2130_init(tmc2130_handle_t *handle, const tmc2130_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Store configuration
    handle->pin_cs = config->pin_cs;
    handle->pin_en = config->pin_en;
    handle->pin_step = config->pin_step;
    handle->pin_dir = config->pin_dir;
    handle->r_sense = config->r_sense;

    // Initialize default register values
    handle->gconf = 0;
    handle->chopconf = 0;
    handle->coolconf = 0;
    handle->pwmconf = 0;
    handle->ihold_irun = 0;

    // Configure CS pin as GPIO output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin");
        return ret;
    }
    gpio_set_level(config->pin_cs, 1);  // CS inactive (high)

    // Configure EN pin if used
    if (config->pin_en >= 0) {
        io_conf.pin_bit_mask = (1ULL << config->pin_en);
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure EN pin");
            return ret;
        }
        gpio_set_level(config->pin_en, 1);  // Disabled by default
    }

    // Configure STEP pin if used
    if (config->pin_step >= 0) {
        io_conf.pin_bit_mask = (1ULL << config->pin_step);
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure STEP pin");
            return ret;
        }
        gpio_set_level(config->pin_step, 0);
    }

    // Configure DIR pin if used
    if (config->pin_dir >= 0) {
        io_conf.pin_bit_mask = (1ULL << config->pin_dir);
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure DIR pin");
            return ret;
        }
        gpio_set_level(config->pin_dir, 0);
    }

    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 5,
    };

    ret = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means bus is already initialized, which is OK
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 3,  // SPI mode 3: CPOL=1, CPHA=1
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = config->spi_clock_speed_hz > 0 ? config->spi_clock_speed_hz : 1000000,
        .input_delay_ns = 0,
        .spics_io_num = -1,  // We handle CS manually
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(config->spi_host, &dev_config, &handle->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Small delay after initialization
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read initial GCONF
    tmc2130_read_reg(handle, TMC2130_REG_GCONF, &handle->gconf);
    tmc2130_read_reg(handle, TMC2130_REG_CHOPCONF, &handle->chopconf);

    ESP_LOGI(TAG, "TMC2130 initialized");
    return ESP_OK;
}

esp_err_t tmc2130_deinit(tmc2130_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = spi_bus_remove_device(handle->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device");
        return ret;
    }

    return ESP_OK;
}

esp_err_t tmc2130_write_reg(tmc2130_handle_t *handle, uint8_t reg, uint32_t data)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_data[5];
    uint8_t rx_data[5];

    tx_data[0] = reg | TMC2130_WRITE_BIT;
    tx_data[1] = (data >> 24) & 0xFF;
    tx_data[2] = (data >> 16) & 0xFF;
    tx_data[3] = (data >> 8) & 0xFF;
    tx_data[4] = data & 0xFF;

    spi_transaction_t trans = {
        .length = 40,  // 5 bytes = 40 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    gpio_set_level(handle->pin_cs, 0);  // CS active
    esp_err_t ret = spi_device_transmit(handle->spi_handle, &trans);
    gpio_set_level(handle->pin_cs, 1);  // CS inactive

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t tmc2130_read_reg(tmc2130_handle_t *handle, uint8_t reg, uint32_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_data[5] = {0};
    uint8_t rx_data[5] = {0};
    esp_err_t ret;

    // First transaction: send register address
    tx_data[0] = reg & 0x7F;  // Read bit is 0

    spi_transaction_t trans = {
        .length = 40,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    gpio_set_level(handle->pin_cs, 0);
    ret = spi_device_transmit(handle->spi_handle, &trans);
    gpio_set_level(handle->pin_cs, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read (1st) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Second transaction: read the data
    memset(tx_data, 0, sizeof(tx_data));
    memset(rx_data, 0, sizeof(rx_data));

    gpio_set_level(handle->pin_cs, 0);
    ret = spi_device_transmit(handle->spi_handle, &trans);
    gpio_set_level(handle->pin_cs, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read (2nd) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    *data = ((uint32_t)rx_data[1] << 24) |
            ((uint32_t)rx_data[2] << 16) |
            ((uint32_t)rx_data[3] << 8) |
            (uint32_t)rx_data[4];

    return ESP_OK;
}

void tmc2130_enable(tmc2130_handle_t *handle)
{
    if (handle != NULL && handle->pin_en >= 0) {
        gpio_set_level(handle->pin_en, 0);  // Active low
    }
}

void tmc2130_disable(tmc2130_handle_t *handle)
{
    if (handle != NULL && handle->pin_en >= 0) {
        gpio_set_level(handle->pin_en, 1);  // Inactive high
    }
}

void tmc2130_step(tmc2130_handle_t *handle)
{
    if (handle != NULL && handle->pin_step >= 0) {
        gpio_set_level(handle->pin_step, 1);
        esp_rom_delay_us(2);  // Minimum pulse width
        gpio_set_level(handle->pin_step, 0);
    }
}

void tmc2130_set_direction(tmc2130_handle_t *handle, bool dir)
{
    if (handle != NULL && handle->pin_dir >= 0) {
        gpio_set_level(handle->pin_dir, dir ? 1 : 0);
    }
}

// GCONF register functions
esp_err_t tmc2130_set_gconf(tmc2130_handle_t *handle, uint32_t gconf)
{
    handle->gconf = gconf;
    return tmc2130_write_reg(handle, TMC2130_REG_GCONF, gconf);
}

esp_err_t tmc2130_get_gconf(tmc2130_handle_t *handle, uint32_t *gconf)
{
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_GCONF, gconf);
    if (ret == ESP_OK) {
        handle->gconf = *gconf;
    }
    return ret;
}

static esp_err_t set_gconf_bit(tmc2130_handle_t *handle, uint8_t pos, bool enable)
{
    if (enable) {
        handle->gconf |= (1UL << pos);
    } else {
        handle->gconf &= ~(1UL << pos);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_GCONF, handle->gconf);
}

esp_err_t tmc2130_set_i_scale_analog(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_I_SCALE_ANALOG_POS, enable);
}

esp_err_t tmc2130_set_internal_rsense(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_INTERNAL_RSENSE_POS, enable);
}

esp_err_t tmc2130_set_en_pwm_mode(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_EN_PWM_MODE_POS, enable);
}

esp_err_t tmc2130_set_enc_commutation(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_ENC_COMMUTATION_POS, enable);
}

esp_err_t tmc2130_set_shaft(tmc2130_handle_t *handle, bool reverse)
{
    return set_gconf_bit(handle, GCONF_SHAFT_POS, reverse);
}

esp_err_t tmc2130_set_diag0_error(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG0_ERROR_POS, enable);
}

esp_err_t tmc2130_set_diag0_otpw(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG0_OTPW_POS, enable);
}

esp_err_t tmc2130_set_diag0_stall(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG0_STALL_POS, enable);
}

esp_err_t tmc2130_set_diag1_stall(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG1_STALL_POS, enable);
}

esp_err_t tmc2130_set_diag1_index(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG1_INDEX_POS, enable);
}

esp_err_t tmc2130_set_diag1_onstate(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG1_ONSTATE_POS, enable);
}

esp_err_t tmc2130_set_diag1_steps_skipped(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG1_STEPS_SKIPPED_POS, enable);
}

esp_err_t tmc2130_set_diag0_int_pushpull(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG0_INT_PUSHPULL_POS, enable);
}

esp_err_t tmc2130_set_diag1_pushpull(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIAG1_PUSHPULL_POS, enable);
}

esp_err_t tmc2130_set_small_hysteresis(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_SMALL_HYSTERESIS_POS, enable);
}

esp_err_t tmc2130_set_stop_enable(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_STOP_ENABLE_POS, enable);
}

esp_err_t tmc2130_set_direct_mode(tmc2130_handle_t *handle, bool enable)
{
    return set_gconf_bit(handle, GCONF_DIRECT_MODE_POS, enable);
}

// GSTAT register functions
esp_err_t tmc2130_get_gstat(tmc2130_handle_t *handle, uint8_t *gstat)
{
    uint32_t data;
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_GSTAT, &data);
    if (ret == ESP_OK) {
        *gstat = data & 0x07;
    }
    return ret;
}

bool tmc2130_is_reset(tmc2130_handle_t *handle)
{
    uint8_t gstat;
    if (tmc2130_get_gstat(handle, &gstat) == ESP_OK) {
        return (gstat & 0x01) != 0;
    }
    return false;
}

bool tmc2130_is_driver_error(tmc2130_handle_t *handle)
{
    uint8_t gstat;
    if (tmc2130_get_gstat(handle, &gstat) == ESP_OK) {
        return (gstat & 0x02) != 0;
    }
    return false;
}

bool tmc2130_is_uv_cp(tmc2130_handle_t *handle)
{
    uint8_t gstat;
    if (tmc2130_get_gstat(handle, &gstat) == ESP_OK) {
        return (gstat & 0x04) != 0;
    }
    return false;
}

// IOIN register functions
esp_err_t tmc2130_get_ioin(tmc2130_handle_t *handle, uint32_t *ioin)
{
    return tmc2130_read_reg(handle, TMC2130_REG_IOIN, ioin);
}

// Current control (IHOLD_IRUN)
esp_err_t tmc2130_set_ihold_irun(tmc2130_handle_t *handle, uint8_t ihold, uint8_t irun, uint8_t iholddelay)
{
    handle->ihold_irun = ((uint32_t)(iholddelay & 0x0F) << 16) |
                         ((uint32_t)(irun & 0x1F) << 8) |
                         (uint32_t)(ihold & 0x1F);
    return tmc2130_write_reg(handle, TMC2130_REG_IHOLD_IRUN, handle->ihold_irun);
}

esp_err_t tmc2130_set_ihold(tmc2130_handle_t *handle, uint8_t ihold)
{
    handle->ihold_irun = (handle->ihold_irun & 0xFFFFFFE0) | (ihold & 0x1F);
    return tmc2130_write_reg(handle, TMC2130_REG_IHOLD_IRUN, handle->ihold_irun);
}

esp_err_t tmc2130_set_irun(tmc2130_handle_t *handle, uint8_t irun)
{
    handle->ihold_irun = (handle->ihold_irun & 0xFFFFE0FF) | ((uint32_t)(irun & 0x1F) << 8);
    return tmc2130_write_reg(handle, TMC2130_REG_IHOLD_IRUN, handle->ihold_irun);
}

esp_err_t tmc2130_set_iholddelay(tmc2130_handle_t *handle, uint8_t iholddelay)
{
    handle->ihold_irun = (handle->ihold_irun & 0xFFF0FFFF) | ((uint32_t)(iholddelay & 0x0F) << 16);
    return tmc2130_write_reg(handle, TMC2130_REG_IHOLD_IRUN, handle->ihold_irun);
}

esp_err_t tmc2130_set_rms_current(tmc2130_handle_t *handle, uint16_t mA, float hold_multiplier)
{
    // Calculate current scaling
    // I_rms = (CS + 1) / 32 * V_fs / R_sense * 1/sqrt(2)
    // V_fs = 0.325V (vsense=1) or 0.180V (vsense=0)
    
    float v_fs = 0.325f;  // Start with high sensitivity
    bool vsense = true;
    
    // Calculate CS for high sensitivity
    float cs_float = (32.0f * 1.41421f * (float)mA * handle->r_sense / 1000.0f / v_fs) - 1.0f;
    
    if (cs_float >= 32.0f) {
        // Need lower sensitivity
        v_fs = 0.180f;
        vsense = false;
        cs_float = (32.0f * 1.41421f * (float)mA * handle->r_sense / 1000.0f / v_fs) - 1.0f;
    }
    
    uint8_t cs = (uint8_t)(cs_float + 0.5f);
    if (cs > 31) cs = 31;
    
    // Set vsense
    tmc2130_set_vsense(handle, vsense);
    
    // Set irun and ihold
    uint8_t ihold = (uint8_t)(cs * hold_multiplier + 0.5f);
    if (ihold > 31) ihold = 31;
    
    return tmc2130_set_ihold_irun(handle, ihold, cs, 10);
}

// Power down delay
esp_err_t tmc2130_set_tpowerdown(tmc2130_handle_t *handle, uint8_t delay)
{
    return tmc2130_write_reg(handle, TMC2130_REG_TPOWERDOWN, delay);
}

// TSTEP register (read only)
esp_err_t tmc2130_get_tstep(tmc2130_handle_t *handle, uint32_t *tstep)
{
    return tmc2130_read_reg(handle, TMC2130_REG_TSTEP, tstep);
}

// Threshold speed settings
esp_err_t tmc2130_set_tpwmthrs(tmc2130_handle_t *handle, uint32_t threshold)
{
    return tmc2130_write_reg(handle, TMC2130_REG_TPWMTHRS, threshold & 0xFFFFF);
}

esp_err_t tmc2130_set_tcoolthrs(tmc2130_handle_t *handle, uint32_t threshold)
{
    return tmc2130_write_reg(handle, TMC2130_REG_TCOOLTHRS, threshold & 0xFFFFF);
}

esp_err_t tmc2130_set_thigh(tmc2130_handle_t *handle, uint32_t threshold)
{
    return tmc2130_write_reg(handle, TMC2130_REG_THIGH, threshold & 0xFFFFF);
}

// CHOPCONF register functions
esp_err_t tmc2130_set_chopconf(tmc2130_handle_t *handle, uint32_t chopconf)
{
    handle->chopconf = chopconf;
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, chopconf);
}

esp_err_t tmc2130_get_chopconf(tmc2130_handle_t *handle, uint32_t *chopconf)
{
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_CHOPCONF, chopconf);
    if (ret == ESP_OK) {
        handle->chopconf = *chopconf;
    }
    return ret;
}

esp_err_t tmc2130_set_toff(tmc2130_handle_t *handle, uint8_t toff)
{
    handle->chopconf = (handle->chopconf & ~CHOPCONF_TOFF_MASK) | (toff & 0x0F);
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_hstrt(tmc2130_handle_t *handle, uint8_t hstrt)
{
    handle->chopconf = (handle->chopconf & ~CHOPCONF_HSTRT_MASK) | ((uint32_t)(hstrt & 0x07) << CHOPCONF_HSTRT_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_hend(tmc2130_handle_t *handle, int8_t hend)
{
    // hend is -3 to 12, stored as 0 to 15
    uint8_t val = (uint8_t)(hend + 3);
    handle->chopconf = (handle->chopconf & ~CHOPCONF_HEND_MASK) | ((uint32_t)(val & 0x0F) << CHOPCONF_HEND_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_tbl(tmc2130_handle_t *handle, uint8_t tbl)
{
    handle->chopconf = (handle->chopconf & ~CHOPCONF_TBL_MASK) | ((uint32_t)(tbl & 0x03) << CHOPCONF_TBL_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_vsense(tmc2130_handle_t *handle, bool high_sensitivity)
{
    if (high_sensitivity) {
        handle->chopconf |= (1UL << CHOPCONF_VSENSE_POS);
    } else {
        handle->chopconf &= ~(1UL << CHOPCONF_VSENSE_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_mres(tmc2130_handle_t *handle, uint8_t mres)
{
    handle->chopconf = (handle->chopconf & ~CHOPCONF_MRES_MASK) | ((uint32_t)(mres & 0x0F) << CHOPCONF_MRES_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_microsteps(tmc2130_handle_t *handle, uint16_t microsteps)
{
    uint8_t mres;
    switch (microsteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; break;  // Default to 16 microsteps
    }
    return tmc2130_set_mres(handle, mres);
}

esp_err_t tmc2130_set_intpol(tmc2130_handle_t *handle, bool enable)
{
    if (enable) {
        handle->chopconf |= (1UL << CHOPCONF_INTPOL_POS);
    } else {
        handle->chopconf &= ~(1UL << CHOPCONF_INTPOL_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_dedge(tmc2130_handle_t *handle, bool enable)
{
    if (enable) {
        handle->chopconf |= (1UL << CHOPCONF_DEDGE_POS);
    } else {
        handle->chopconf &= ~(1UL << CHOPCONF_DEDGE_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_diss2g(tmc2130_handle_t *handle, bool disable)
{
    if (disable) {
        handle->chopconf |= (1UL << CHOPCONF_DISS2G_POS);
    } else {
        handle->chopconf &= ~(1UL << CHOPCONF_DISS2G_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

esp_err_t tmc2130_set_chm(tmc2130_handle_t *handle, bool spread_cycle)
{
    if (spread_cycle) {
        handle->chopconf |= (1UL << CHOPCONF_CHM_POS);
    } else {
        handle->chopconf &= ~(1UL << CHOPCONF_CHM_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_CHOPCONF, handle->chopconf);
}

// COOLCONF register functions (StallGuard2 and CoolStep)
esp_err_t tmc2130_set_coolconf(tmc2130_handle_t *handle, uint32_t coolconf)
{
    handle->coolconf = coolconf;
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, coolconf);
}

esp_err_t tmc2130_get_coolconf(tmc2130_handle_t *handle, uint32_t *coolconf)
{
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_COOLCONF, coolconf);
    if (ret == ESP_OK) {
        handle->coolconf = *coolconf;
    }
    return ret;
}

esp_err_t tmc2130_set_semin(tmc2130_handle_t *handle, uint8_t semin)
{
    handle->coolconf = (handle->coolconf & ~0x0F) | (semin & 0x0F);
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

esp_err_t tmc2130_set_seup(tmc2130_handle_t *handle, uint8_t seup)
{
    handle->coolconf = (handle->coolconf & ~(0x03 << COOLCONF_SEUP_POS)) | ((uint32_t)(seup & 0x03) << COOLCONF_SEUP_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

esp_err_t tmc2130_set_semax(tmc2130_handle_t *handle, uint8_t semax)
{
    handle->coolconf = (handle->coolconf & ~(0x0F << COOLCONF_SEMAX_POS)) | ((uint32_t)(semax & 0x0F) << COOLCONF_SEMAX_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

esp_err_t tmc2130_set_sedn(tmc2130_handle_t *handle, uint8_t sedn)
{
    handle->coolconf = (handle->coolconf & ~(0x03 << COOLCONF_SEDN_POS)) | ((uint32_t)(sedn & 0x03) << COOLCONF_SEDN_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

esp_err_t tmc2130_set_seimin(tmc2130_handle_t *handle, bool half_cs)
{
    if (half_cs) {
        handle->coolconf |= (1UL << COOLCONF_SEIMIN_POS);
    } else {
        handle->coolconf &= ~(1UL << COOLCONF_SEIMIN_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

esp_err_t tmc2130_set_sgt(tmc2130_handle_t *handle, int8_t sgt)
{
    // sgt is -64 to 63, stored as unsigned with offset
    uint32_t val = (uint32_t)(sgt & 0x7F);
    handle->coolconf = (handle->coolconf & ~(0x7F << COOLCONF_SGT_POS)) | (val << COOLCONF_SGT_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

esp_err_t tmc2130_set_sfilt(tmc2130_handle_t *handle, bool enable)
{
    if (enable) {
        handle->coolconf |= (1UL << COOLCONF_SFILT_POS);
    } else {
        handle->coolconf &= ~(1UL << COOLCONF_SFILT_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_COOLCONF, handle->coolconf);
}

// PWMCONF register functions (StealthChop)
esp_err_t tmc2130_set_pwmconf(tmc2130_handle_t *handle, uint32_t pwmconf)
{
    handle->pwmconf = pwmconf;
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, pwmconf);
}

esp_err_t tmc2130_get_pwmconf(tmc2130_handle_t *handle, uint32_t *pwmconf)
{
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_PWMCONF, pwmconf);
    if (ret == ESP_OK) {
        handle->pwmconf = *pwmconf;
    }
    return ret;
}

esp_err_t tmc2130_set_pwm_ampl(tmc2130_handle_t *handle, uint8_t ampl)
{
    handle->pwmconf = (handle->pwmconf & ~0xFF) | ampl;
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, handle->pwmconf);
}

esp_err_t tmc2130_set_pwm_grad(tmc2130_handle_t *handle, uint8_t grad)
{
    handle->pwmconf = (handle->pwmconf & ~(0xFF << PWMCONF_PWM_GRAD_POS)) | ((uint32_t)grad << PWMCONF_PWM_GRAD_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, handle->pwmconf);
}

esp_err_t tmc2130_set_pwm_freq(tmc2130_handle_t *handle, uint8_t freq)
{
    handle->pwmconf = (handle->pwmconf & ~(0x03 << PWMCONF_PWM_FREQ_POS)) | ((uint32_t)(freq & 0x03) << PWMCONF_PWM_FREQ_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, handle->pwmconf);
}

esp_err_t tmc2130_set_pwm_autoscale(tmc2130_handle_t *handle, bool enable)
{
    if (enable) {
        handle->pwmconf |= (1UL << PWMCONF_PWM_AUTOSCALE_POS);
    } else {
        handle->pwmconf &= ~(1UL << PWMCONF_PWM_AUTOSCALE_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, handle->pwmconf);
}

esp_err_t tmc2130_set_pwm_symmetric(tmc2130_handle_t *handle, bool enable)
{
    if (enable) {
        handle->pwmconf |= (1UL << PWMCONF_PWM_SYMMETRIC_POS);
    } else {
        handle->pwmconf &= ~(1UL << PWMCONF_PWM_SYMMETRIC_POS);
    }
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, handle->pwmconf);
}

esp_err_t tmc2130_set_freewheel(tmc2130_handle_t *handle, uint8_t mode)
{
    handle->pwmconf = (handle->pwmconf & ~(0x03 << PWMCONF_FREEWHEEL_POS)) | ((uint32_t)(mode & 0x03) << PWMCONF_FREEWHEEL_POS);
    return tmc2130_write_reg(handle, TMC2130_REG_PWMCONF, handle->pwmconf);
}

// DRV_STATUS register functions (read only)
esp_err_t tmc2130_get_drv_status(tmc2130_handle_t *handle, uint32_t *status)
{
    return tmc2130_read_reg(handle, TMC2130_REG_DRV_STATUS, status);
}

esp_err_t tmc2130_get_drv_status_struct(tmc2130_handle_t *handle, tmc2130_drv_status_t *status)
{
    uint32_t data;
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_DRV_STATUS, &data);
    if (ret == ESP_OK) {
        memcpy(status, &data, sizeof(tmc2130_drv_status_t));
    }
    return ret;
}

uint16_t tmc2130_get_sg_result(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return status & 0x3FF;
    }
    return 0;
}

bool tmc2130_is_fsactive(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 15)) != 0;
    }
    return false;
}

uint8_t tmc2130_get_cs_actual(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status >> 16) & 0x1F;
    }
    return 0;
}

bool tmc2130_is_stallguard(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 24)) != 0;
    }
    return false;
}

bool tmc2130_is_ot(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 25)) != 0;
    }
    return false;
}

bool tmc2130_is_otpw(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 26)) != 0;
    }
    return false;
}

bool tmc2130_is_s2ga(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 27)) != 0;
    }
    return false;
}

bool tmc2130_is_s2gb(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 28)) != 0;
    }
    return false;
}

bool tmc2130_is_ola(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 29)) != 0;
    }
    return false;
}

bool tmc2130_is_olb(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 30)) != 0;
    }
    return false;
}

bool tmc2130_is_stst(tmc2130_handle_t *handle)
{
    uint32_t status;
    if (tmc2130_get_drv_status(handle, &status) == ESP_OK) {
        return (status & (1UL << 31)) != 0;
    }
    return false;
}

// PWM_SCALE register (read only)
esp_err_t tmc2130_get_pwm_scale(tmc2130_handle_t *handle, uint8_t *scale)
{
    uint32_t data;
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_PWM_SCALE, &data);
    if (ret == ESP_OK) {
        *scale = data & 0xFF;
    }
    return ret;
}

// MSCNT register (read only)
esp_err_t tmc2130_get_mscnt(tmc2130_handle_t *handle, uint16_t *mscnt)
{
    uint32_t data;
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_MSCNT, &data);
    if (ret == ESP_OK) {
        *mscnt = data & 0x3FF;
    }
    return ret;
}

// MSCURACT register (read only)
esp_err_t tmc2130_get_mscuract(tmc2130_handle_t *handle, int16_t *cur_a, int16_t *cur_b)
{
    uint32_t data;
    esp_err_t ret = tmc2130_read_reg(handle, TMC2130_REG_MSCURACT, &data);
    if (ret == ESP_OK) {
        // Sign extend 9-bit values
        int16_t a = data & 0x1FF;
        if (a & 0x100) a |= 0xFE00;  // Sign extend
        
        int16_t b = (data >> 16) & 0x1FF;
        if (b & 0x100) b |= 0xFE00;  // Sign extend
        
        if (cur_a) *cur_a = a;
        if (cur_b) *cur_b = b;
    }
    return ret;
}

// LOST_STEPS register (read only)
esp_err_t tmc2130_get_lost_steps(tmc2130_handle_t *handle, uint32_t *steps)
{
    return tmc2130_read_reg(handle, TMC2130_REG_LOST_STEPS, steps);
}

// ENCM_CTRL register
esp_err_t tmc2130_set_encm_ctrl(tmc2130_handle_t *handle, bool inv, bool maxspeed)
{
    uint32_t data = 0;
    if (inv) data |= 0x01;
    if (maxspeed) data |= 0x02;
    return tmc2130_write_reg(handle, TMC2130_REG_ENCM_CTRL, data);
}

// Utility functions
uint32_t tmc2130_version(tmc2130_handle_t *handle)
{
    uint32_t ioin;
    if (tmc2130_get_ioin(handle, &ioin) == ESP_OK) {
        return (ioin >> 24) & 0xFF;
    }
    return 0;
}

bool tmc2130_test_connection(tmc2130_handle_t *handle)
{
    uint32_t version = tmc2130_version(handle);
    // TMC2130 should return version 0x11
    if (version == 0x11) {
        ESP_LOGI(TAG, "TMC2130 detected, version: 0x%02lX", version);
        return true;
    }
    ESP_LOGW(TAG, "TMC2130 not detected or wrong version: 0x%02lX (expected 0x11)", version);
    return false;
}