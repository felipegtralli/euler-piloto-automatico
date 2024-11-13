#include "euler_wifi_ap.h"

#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_log.h"

#include "euler_macros.h"

static const char* TAG = "wifi-ap";

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

static esp_err_t nvs_get_wifi_config(const char* key, wifi_config_t* cfg) {
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if(event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if(event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void euler_wifi_init_ap() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    ESP_ERROR_CHECK(nvs_get_wifi_config(WIFI_NVS_CONFIG, &wifi_config));
    
    if(strlen((const char*) wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);
}