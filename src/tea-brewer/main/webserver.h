/**
 * @file webserver.h
 * @brief Web server dashboard for Tea Brewer
 * 
 * This module provides a web-based dashboard for controlling the tea brewer,
 * viewing temperature, and scheduling brews.
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the web server
 * @return ESP_OK on success
 */
esp_err_t webserver_init(void);

/**
 * @brief Stop the web server
 * @return ESP_OK on success
 */
esp_err_t webserver_stop(void);

/**
 * @brief Check if web server is running
 * @return true if running
 */
bool webserver_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // WEBSERVER_H
