#pragma once

#include <string.h>

#include "euler_pid_control.h"

#include "esp_http_server.h"
#include "esp_err.h"

/**
 * @brief Monitor structure
 * 
 */
typedef struct {
    euler_pid_control_t* pid;
    double pulses;
    double duty_cycle;
    double rpm;
} euler_monitor_t;

/**
 * @brief Root get handler
 * 
 * @param req httpd request
 */
esp_err_t root_get_handler(httpd_req_t* req);

/**
 * @brief Monitor get handler
 * 
 * @param req httpd request
 */
esp_err_t monitor_get_handler(httpd_req_t* req);

/**
 * @brief PID put handler
 * 
 * @param req httpd request
 */
esp_err_t pid_put_handler(httpd_req_t* req);