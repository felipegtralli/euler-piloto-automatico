#ifndef EULER_NVS_HELPER_H
#define EULER_NVS_HELPER_H

#include "euler_pid_control.h"
#include "euler_filter.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

esp_err_t nvs_get_pid_params(const char* key, euler_pid_params_t* params);
esp_err_t nvs_get_filter_params(const char* key, euler_filter_params_t* params);
esp_err_t nvs_get_wifi_config(const char* key, wifi_config_t* cfg);

esp_err_t nvs_set_pid_params(const char* key, euler_pid_params_t* params);
esp_err_t nvs_set_filter_params(const char* key, euler_filter_params_t* params);
esp_err_t nvs_set_wifi_config(const char* key, wifi_config_t* cfg);

#endif // EULER_NVS_HELPER_H