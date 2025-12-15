/**
 * @file console.h
 * @brief USB Serial JTAG console for human interface
 */

#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize USB Serial JTAG console
 */
esp_err_t console_init(void);

/**
 * @brief Print formatted string to console
 */
void console_printf(const char *fmt, ...);

/**
 * @brief Print string to console
 */
void console_print(const char *str);

/**
 * @brief Check if console has input available (non-blocking)
 */
bool console_has_input(void);

/**
 * @brief Start console task (handles human commands)
 */
void console_start_task(void);

#endif // CONSOLE_H