#pragma once

#include "euler_pid_control.h"
#include "euler_filter.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

esp_err_t nvs_get_pid_params(const char* key, euler_pid_params_t* params);
esp_err_t nvs_get_filter_config(const char* key, euler_filter_config_t* config);
esp_err_t nvs_set_pid_params(const char* key, euler_pid_params_t* params);

double constrain(double x, double min, double max);
double map_2pwm(double val, double min, double max);