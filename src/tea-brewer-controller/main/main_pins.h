/**
 * @file pins.h
 * @brief Pin definitions for Tea Brewer Controller
 */

#ifndef PINS_H
#define PINS_H

/* ============================================
   TMC2130 STEPPER DRIVER PINS
   ============================================ */
#define PIN_MOTOR_EN        2
#define PIN_MOTOR_DIR       0
#define PIN_MOTOR_STEP      1
#define PIN_MOTOR_CS        21

/* ============================================
   SPI PINS (for TMC2130)
   ============================================ */
#define PIN_SPI_SCLK        19
#define PIN_SPI_MOSI        18
#define PIN_SPI_MISO        20

/* ============================================
   UART0 PINS (ESP-to-ESP communication)
   ============================================ */
#define PIN_UART_TX         16
#define PIN_UART_RX         17

/* ============================================
   I2C PINS (for MLX90614 Temperature Sensor)
   ============================================ */
#define PIN_I2C_SDA         22
#define PIN_I2C_SCL         23

#endif // PINS_H