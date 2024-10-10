#include "euler_tasks.h"
#include "euler_sensor.h"
#include "euler_wifi.h"
#include "euler_server.h"

#include "esp_log.h"

static const char* CTRL_TAG = "MAIN-CTRL";
static const char* SERVER_TAG = "SERVER";

extern TaskHandle_t ctrl_task_handle;
extern QueueHandle_t update_ctrl_queue;

void ctrl_task(void* pvParameters) {
    ctrl_loop_context_t* ctx = (ctrl_loop_context_t*) pvParameters;
    ESP_LOGI(CTRL_TAG, "control task started");

    while(true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(xQueueReceive(update_ctrl_queue, &ctx->pid_params, 0) == pdTRUE) {
            ESP_ERROR_CHECK(euler_pid_update_config(&ctx->pid, &ctx->pid_params));
            ESP_LOGI(CTRL_TAG, "pid params updated");
        }

        float sample = euler_read_sensor();

        ESP_ERROR_CHECK(euler_filter_butter_2order_low(&ctx->filter, &sample));

        float pid_output = 0.0;
        ESP_ERROR_CHECK(euler_pid_compute(&ctx->pid, sample, &pid_output));
    }
}

bool IRAM_ATTR ctrl_task_timer_handle(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(ctrl_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
        return true;
    }
    return false;
}

void tcp_server_task(void* pvParameters) {
    tcp_server_context_t* ctx = (tcp_server_context_t*) pvParameters;
    char addr_str[128] = {0};
    struct sockaddr_in dest_addr;

    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if(!netif) {
        ESP_LOGE(SERVER_TAG, "netif failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip_info));

    dest_addr.sin_addr.s_addr = ip_info.ip.addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(listen_sock < 0) {
        ESP_LOGE(SERVER_TAG, "failed to create socket");
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(SERVER_TAG, "socket created");

    if(bind(listen_sock, (struct sockaddr*) &dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(SERVER_TAG, "failed to bind socket");
        goto CLEAN_UP;
    } else {
        char ip[32] = {0};
        snprintf(ip, sizeof(ip) - 1, IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(SERVER_TAG, "socket bound ip: %s port %d", ip, PORT);
    }

    if(listen(listen_sock, 1) != 0) {
        ESP_LOGE(SERVER_TAG, "failed to listen on socket");
        goto CLEAN_UP;
    }

    while(true) {
        ESP_LOGI(SERVER_TAG, "waiting for connection...");

        struct sockaddr_in source_addr;
        socklen_t len = sizeof(source_addr);

        int sock = accept(listen_sock, (struct sockaddr*) &source_addr, &len);
        if(sock < 0) {
            ESP_LOGE(SERVER_TAG, "failed to accept connection");
            break;
        }
        inet_ntoa_r(((struct sockaddr_in*) &source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);    
        ESP_LOGI(SERVER_TAG, "accepted connection from %s", addr_str);

        while(euler_receive_data(sock, &ctx->pid_params));

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}