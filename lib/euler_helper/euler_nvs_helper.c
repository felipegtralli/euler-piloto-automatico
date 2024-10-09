#include "euler_nvs_helper.h"

#include "esp_err.h"
#include "esp_log.h"

const char* TAG = "NVS-HELPER";

float nvs_get_float(nvs_handle_t handle, const char* key, float default_value) {
    size_t required_size = sizeof(char) * 32;
    char buffer[32];
    esp_err_t err = nvs_get_str(handle, key, buffer, &required_size);

    switch(err) {
        case ESP_OK:
            ESP_LOGI(TAG, "nvs_get_float: %s = %s", key, buffer);
            return strtof(buffer, NULL);
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "nvs_get_float: %s not found, using default value", key);
            return default_value;
        default:
            ESP_ERROR_CHECK(err);
            return default_value;
    }
}