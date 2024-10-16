#include "euler_nvs_helper.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "euler_macros.h"

static const char* TAG = "NVS-HELPER";

static void use_default_pid(euler_pid_params_t* params) {
    params->kp = 5.0;
    params->ki = 3.0;
    params->kd = 1.5;
    params->setpoint = 3500.0;
    params->max_output = 7000.0;
}

static void use_default_filter(euler_filter_params_t* params) {
    params->cutoff = 233.33;
    params->fs = SAMPLING_INTERVAL_HZ;
}

static void use_default_wifi(wifi_config_t* cfg) {
    char ssid[32] = "esp32-AP";
    char password[64] = "";

    strncpy((char*)cfg->ap.ssid, ssid, sizeof(cfg->ap.ssid));
    cfg->ap.ssid_len = strlen(ssid);
    cfg->ap.channel = 1;
    strncpy((char*)cfg->ap.password, password, sizeof(cfg->ap.password));
    cfg->ap.max_connection = 4;
    cfg->ap.authmode = WIFI_AUTH_WPA2_PSK;
    cfg->ap.pmf_cfg.required = true;
}

esp_err_t nvs_get_pid_params(const char* key, euler_pid_params_t* params) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(PID_NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*params);
    err = nvs_get_blob(nvs_handle, key, params, &required_size);
    switch(err) {
        case ESP_OK:
            ESP_LOGI(TAG, "pid params found");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "pid params not found... using default values");
            use_default_pid(params);
            err = ESP_OK;
            break;
        default:
            break;
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_get_filter_params(const char* key, euler_filter_params_t* params) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(FILTER_NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*params);
    err = nvs_get_blob(nvs_handle, key, params, &required_size);
    switch(err) {
        case ESP_OK:
            ESP_LOGI(TAG, "filter params found");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "filter params not found... using default values");
            use_default_filter(params);
            err = ESP_OK;
            break;
        default:
            break;
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_get_wifi_config(const char* key, wifi_config_t* cfg) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*cfg);
    err = nvs_get_blob(nvs_handle, key, cfg, &required_size);
    switch(err) {
        case ESP_OK:
            ESP_LOGI(TAG, "wifi config found");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "wifi config not found... using default values");
            use_default_wifi(cfg);
            err = ESP_OK;
            break;
        default:
            break;
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_set_pid_params(const char* key, euler_pid_params_t* params) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(PID_NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*params);
    err = nvs_set_blob(nvs_handle, key, params, required_size);
    if(err != ESP_OK) {
        goto ERROR;
    } else {
        err = nvs_commit(nvs_handle);
        if(err != ESP_OK) {
            goto ERROR;
        }
    }
    nvs_close(nvs_handle);
    return ESP_OK;

ERROR:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_set_filter_params(const char* key, euler_filter_params_t* params) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(FILTER_NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*params);
    err = nvs_set_blob(nvs_handle, key, params, required_size);
    if(err != ESP_OK) {
        goto ERROR;
    } else {
        err = nvs_commit(nvs_handle);
        if(err != ESP_OK) {
            goto ERROR;
        }
    }
    nvs_close(nvs_handle);
    return ESP_OK;

ERROR:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_set_wifi_config(const char* key, wifi_config_t* cfg) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*cfg);
    err = nvs_set_blob(nvs_handle, key, cfg, required_size);
    if(err != ESP_OK) {
        goto ERROR;
    } else {
        err = nvs_commit(nvs_handle);
        if(err != ESP_OK) {
            goto ERROR;
        }
    }
    nvs_close(nvs_handle);
    return ESP_OK;

ERROR:
    nvs_close(nvs_handle);
    return err;
}