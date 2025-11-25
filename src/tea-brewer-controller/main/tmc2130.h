/**
 * @file tmc2130.h
 * @brief TMC2130 Stepper Motor Driver for ESP-IDF
 * 
 * Ported from Arduino TMC2130Stepper library by teemuatlut
 * Original: https://github.com/teemuatlut/TMC2130Stepper
 */

#ifndef TMC2130_H
#define TMC2130_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// TMC2130 Register Addresses
#define TMC2130_REG_GCONF       0x00
#define TMC2130_REG_GSTAT       0x01
#define TMC2130_REG_IOIN        0x04
#define TMC2130_REG_IHOLD_IRUN  0x10
#define TMC2130_REG_TPOWERDOWN  0x11
#define TMC2130_REG_TSTEP       0x12
#define TMC2130_REG_TPWMTHRS    0x13
#define TMC2130_REG_TCOOLTHRS   0x14
#define TMC2130_REG_THIGH       0x15
#define TMC2130_REG_XDIRECT     0x2D
#define TMC2130_REG_VDCMIN      0x33
#define TMC2130_REG_MSLUT0      0x60
#define TMC2130_REG_MSLUT1      0x61
#define TMC2130_REG_MSLUT2      0x62
#define TMC2130_REG_MSLUT3      0x63
#define TMC2130_REG_MSLUT4      0x64
#define TMC2130_REG_MSLUT5      0x65
#define TMC2130_REG_MSLUT6      0x66
#define TMC2130_REG_MSLUT7      0x67
#define TMC2130_REG_MSLUTSEL    0x68
#define TMC2130_REG_MSLUTSTART  0x69
#define TMC2130_REG_MSCNT       0x6A
#define TMC2130_REG_MSCURACT    0x6B
#define TMC2130_REG_CHOPCONF    0x6C
#define TMC2130_REG_COOLCONF    0x6D
#define TMC2130_REG_DCCTRL      0x6E
#define TMC2130_REG_DRV_STATUS  0x6F
#define TMC2130_REG_PWMCONF     0x70
#define TMC2130_REG_PWM_SCALE   0x71
#define TMC2130_REG_ENCM_CTRL   0x72
#define TMC2130_REG_LOST_STEPS  0x73

// Write bit
#define TMC2130_WRITE_BIT       0x80

/**
 * @brief TMC2130 configuration structure
 */
typedef struct {
    spi_host_device_t spi_host;     // SPI host (SPI2_HOST or SPI3_HOST)
    gpio_num_t pin_mosi;            // MOSI pin
    gpio_num_t pin_miso;            // MISO pin
    gpio_num_t pin_sclk;            // SCLK pin
    gpio_num_t pin_cs;              // Chip Select pin
    gpio_num_t pin_en;              // Enable pin (-1 if not used)
    gpio_num_t pin_step;            // Step pin (-1 if not used)
    gpio_num_t pin_dir;             // Direction pin (-1 if not used)
    int spi_clock_speed_hz;         // SPI clock speed (default: 1MHz)
    float r_sense;                  // Sense resistor value in ohms
} tmc2130_config_t;

/**
 * @brief TMC2130 handle structure
 */
typedef struct {
    spi_device_handle_t spi_handle;
    gpio_num_t pin_cs;
    gpio_num_t pin_en;
    gpio_num_t pin_step;
    gpio_num_t pin_dir;
    float r_sense;
    uint32_t gconf;
    uint32_t chopconf;
    uint32_t coolconf;
    uint32_t pwmconf;
    uint32_t ihold_irun;
} tmc2130_handle_t;

/**
 * @brief DRV_STATUS register structure
 */
typedef struct {
    uint32_t sg_result : 10;
    uint32_t reserved1 : 5;
    uint32_t fsactive : 1;
    uint32_t cs_actual : 5;
    uint32_t reserved2 : 3;
    uint32_t stallguard : 1;
    uint32_t ot : 1;
    uint32_t otpw : 1;
    uint32_t s2ga : 1;
    uint32_t s2gb : 1;
    uint32_t ola : 1;
    uint32_t olb : 1;
    uint32_t stst : 1;
} tmc2130_drv_status_t;

// Initialization and Deinitialization
esp_err_t tmc2130_init(tmc2130_handle_t *handle, const tmc2130_config_t *config);
esp_err_t tmc2130_deinit(tmc2130_handle_t *handle);

// Low-level SPI communication
esp_err_t tmc2130_write_reg(tmc2130_handle_t *handle, uint8_t reg, uint32_t data);
esp_err_t tmc2130_read_reg(tmc2130_handle_t *handle, uint8_t reg, uint32_t *data);

// Enable/Disable motor driver
void tmc2130_enable(tmc2130_handle_t *handle);
void tmc2130_disable(tmc2130_handle_t *handle);

// Step and direction control
void tmc2130_step(tmc2130_handle_t *handle);
void tmc2130_set_direction(tmc2130_handle_t *handle, bool dir);

// GCONF register functions
esp_err_t tmc2130_set_gconf(tmc2130_handle_t *handle, uint32_t gconf);
esp_err_t tmc2130_get_gconf(tmc2130_handle_t *handle, uint32_t *gconf);
esp_err_t tmc2130_set_i_scale_analog(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_internal_rsense(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_en_pwm_mode(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_enc_commutation(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_shaft(tmc2130_handle_t *handle, bool reverse);
esp_err_t tmc2130_set_diag0_error(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag0_otpw(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag0_stall(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag1_stall(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag1_index(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag1_onstate(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag1_steps_skipped(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag0_int_pushpull(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diag1_pushpull(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_small_hysteresis(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_stop_enable(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_direct_mode(tmc2130_handle_t *handle, bool enable);

// GSTAT register functions
esp_err_t tmc2130_get_gstat(tmc2130_handle_t *handle, uint8_t *gstat);
bool tmc2130_is_reset(tmc2130_handle_t *handle);
bool tmc2130_is_driver_error(tmc2130_handle_t *handle);
bool tmc2130_is_uv_cp(tmc2130_handle_t *handle);

// IOIN register functions
esp_err_t tmc2130_get_ioin(tmc2130_handle_t *handle, uint32_t *ioin);

// Current control (IHOLD_IRUN)
esp_err_t tmc2130_set_ihold_irun(tmc2130_handle_t *handle, uint8_t ihold, uint8_t irun, uint8_t iholddelay);
esp_err_t tmc2130_set_ihold(tmc2130_handle_t *handle, uint8_t ihold);
esp_err_t tmc2130_set_irun(tmc2130_handle_t *handle, uint8_t irun);
esp_err_t tmc2130_set_iholddelay(tmc2130_handle_t *handle, uint8_t iholddelay);
esp_err_t tmc2130_set_rms_current(tmc2130_handle_t *handle, uint16_t mA, float hold_multiplier);

// Power down delay
esp_err_t tmc2130_set_tpowerdown(tmc2130_handle_t *handle, uint8_t delay);

// TSTEP register (read only)
esp_err_t tmc2130_get_tstep(tmc2130_handle_t *handle, uint32_t *tstep);

// Threshold speed settings
esp_err_t tmc2130_set_tpwmthrs(tmc2130_handle_t *handle, uint32_t threshold);
esp_err_t tmc2130_set_tcoolthrs(tmc2130_handle_t *handle, uint32_t threshold);
esp_err_t tmc2130_set_thigh(tmc2130_handle_t *handle, uint32_t threshold);

// CHOPCONF register functions
esp_err_t tmc2130_set_chopconf(tmc2130_handle_t *handle, uint32_t chopconf);
esp_err_t tmc2130_get_chopconf(tmc2130_handle_t *handle, uint32_t *chopconf);
esp_err_t tmc2130_set_toff(tmc2130_handle_t *handle, uint8_t toff);
esp_err_t tmc2130_set_hstrt(tmc2130_handle_t *handle, uint8_t hstrt);
esp_err_t tmc2130_set_hend(tmc2130_handle_t *handle, int8_t hend);
esp_err_t tmc2130_set_tbl(tmc2130_handle_t *handle, uint8_t tbl);
esp_err_t tmc2130_set_vsense(tmc2130_handle_t *handle, bool high_sensitivity);
esp_err_t tmc2130_set_mres(tmc2130_handle_t *handle, uint8_t mres);
esp_err_t tmc2130_set_microsteps(tmc2130_handle_t *handle, uint16_t microsteps);
esp_err_t tmc2130_set_intpol(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_dedge(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_diss2g(tmc2130_handle_t *handle, bool disable);
esp_err_t tmc2130_set_chm(tmc2130_handle_t *handle, bool spread_cycle);

// COOLCONF register functions (StallGuard2 and CoolStep)
esp_err_t tmc2130_set_coolconf(tmc2130_handle_t *handle, uint32_t coolconf);
esp_err_t tmc2130_get_coolconf(tmc2130_handle_t *handle, uint32_t *coolconf);
esp_err_t tmc2130_set_semin(tmc2130_handle_t *handle, uint8_t semin);
esp_err_t tmc2130_set_seup(tmc2130_handle_t *handle, uint8_t seup);
esp_err_t tmc2130_set_semax(tmc2130_handle_t *handle, uint8_t semax);
esp_err_t tmc2130_set_sedn(tmc2130_handle_t *handle, uint8_t sedn);
esp_err_t tmc2130_set_seimin(tmc2130_handle_t *handle, bool half_cs);
esp_err_t tmc2130_set_sgt(tmc2130_handle_t *handle, int8_t sgt);
esp_err_t tmc2130_set_sfilt(tmc2130_handle_t *handle, bool enable);

// PWMCONF register functions (StealthChop)
esp_err_t tmc2130_set_pwmconf(tmc2130_handle_t *handle, uint32_t pwmconf);
esp_err_t tmc2130_get_pwmconf(tmc2130_handle_t *handle, uint32_t *pwmconf);
esp_err_t tmc2130_set_pwm_ampl(tmc2130_handle_t *handle, uint8_t ampl);
esp_err_t tmc2130_set_pwm_grad(tmc2130_handle_t *handle, uint8_t grad);
esp_err_t tmc2130_set_pwm_freq(tmc2130_handle_t *handle, uint8_t freq);
esp_err_t tmc2130_set_pwm_autoscale(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_pwm_symmetric(tmc2130_handle_t *handle, bool enable);
esp_err_t tmc2130_set_freewheel(tmc2130_handle_t *handle, uint8_t mode);

// DRV_STATUS register functions (read only)
esp_err_t tmc2130_get_drv_status(tmc2130_handle_t *handle, uint32_t *status);
esp_err_t tmc2130_get_drv_status_struct(tmc2130_handle_t *handle, tmc2130_drv_status_t *status);
uint16_t tmc2130_get_sg_result(tmc2130_handle_t *handle);
bool tmc2130_is_fsactive(tmc2130_handle_t *handle);
uint8_t tmc2130_get_cs_actual(tmc2130_handle_t *handle);
bool tmc2130_is_stallguard(tmc2130_handle_t *handle);
bool tmc2130_is_ot(tmc2130_handle_t *handle);
bool tmc2130_is_otpw(tmc2130_handle_t *handle);
bool tmc2130_is_s2ga(tmc2130_handle_t *handle);
bool tmc2130_is_s2gb(tmc2130_handle_t *handle);
bool tmc2130_is_ola(tmc2130_handle_t *handle);
bool tmc2130_is_olb(tmc2130_handle_t *handle);
bool tmc2130_is_stst(tmc2130_handle_t *handle);

// PWM_SCALE register (read only)
esp_err_t tmc2130_get_pwm_scale(tmc2130_handle_t *handle, uint8_t *scale);

// MSCNT register (read only)
esp_err_t tmc2130_get_mscnt(tmc2130_handle_t *handle, uint16_t *mscnt);

// MSCURACT register (read only)
esp_err_t tmc2130_get_mscuract(tmc2130_handle_t *handle, int16_t *cur_a, int16_t *cur_b);

// LOST_STEPS register (read only)
esp_err_t tmc2130_get_lost_steps(tmc2130_handle_t *handle, uint32_t *steps);

// ENCM_CTRL register
esp_err_t tmc2130_set_encm_ctrl(tmc2130_handle_t *handle, bool inv, bool maxspeed);

// Utility functions
uint32_t tmc2130_version(tmc2130_handle_t *handle);
bool tmc2130_test_connection(tmc2130_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // TMC2130_H