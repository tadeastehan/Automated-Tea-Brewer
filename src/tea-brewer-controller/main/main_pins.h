/**
 * @file pins.h
 * @brief Pin definitions for Tea Brewer Controller
 */

#ifndef PINS_H
#define PINS_H

/* ============================================
   TMC2130 STEPPER DRIVER PINS
   ============================================ */
                           // M1   M2
#define PIN_MOTOR_EN        18 // 2
#define PIN_MOTOR_DIR       19 // 6
#define PIN_MOTOR_STEP      20 // 7
#define PIN_MOTOR_CS        21 // 11 

/* ============================================
   SPI PINS (for TMC2130)
   ============================================ */
#define PIN_SPI_SCLK        10
#define PIN_SPI_MOSI        1
#define PIN_SPI_MISO        0

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

/* ============================================
   INDUCTION COOKER CONTROL PIN
   ============================================ */
#define PIN_INDUCTION       3  // HIGH = ON, LOW = OFF

#endif // PINS_H