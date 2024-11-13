#include "euler_tasks.h"
#include "euler_sensor.h"
#include "euler_wifi_ap.h"
#include "euler_restful_server.h"
#include "euler_helper.h"

#include "esp_log.h"

static const char* CTRL_TAG = "ctrl";
static const char* SERVER_TAG = "server";

static euler_monitor_t monitor = {0};

extern TaskHandle_t ctrl_task_handle;
extern QueueHandle_t update_ctrl_queue;

void ctrl_task(void* pvParameters) {
    ctrl_loop_context_t* ctx = (ctrl_loop_context_t*) pvParameters;
    ESP_LOGI(CTRL_TAG, "control task started");

    while(true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(xQueueReceive(update_ctrl_queue, &ctx->pid_params, 0) == pdTRUE) {
            ESP_ERROR_CHECK(euler_pid_update_config(ctx->pid, &ctx->pid_params));
            ESP_LOGI(CTRL_TAG, "pid params updated");
        }

        double sample_pulses = (double) euler_encoder_get_pulses(ctx->encoder_unit);

        ESP_ERROR_CHECK(euler_filter_exp_mov_avg(ctx->filter, &sample_pulses));

        double pid_output = 0.0;
        ESP_ERROR_CHECK(euler_pid_compute(ctx->pid, sample_pulses, &pid_output));

        double duty_cycle = map_2pwm(pid_output, ctx->pid->_min_output, ctx->pid->_max_output);
        euler_bdc_motor_set_speed(ctx->motor, duty_cycle);

        monitor.pid = ctx->pid;
        monitor.pulses = sample_pulses;
        monitor.duty_cycle = duty_cycle;
        monitor.rpm = euler_pulses2rpm(sample_pulses);
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

void restful_server_task(void* pvParameters) {
    restful_server_context_t* ctx = (restful_server_context_t*) pvParameters;

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(SERVER_TAG, "starting server on port: %d", config.server_port);
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t monitor_uri = {
        .uri = "/api/monitor",
        .method = HTTP_GET,
        .handler = monitor_get_handler,
        .user_ctx = (void*) &monitor,
    };
    httpd_register_uri_handler(server, &monitor_uri);

    httpd_uri_t pid_uri = {
        .uri = "/api/pid",
        .method = HTTP_PUT,
        .handler = pid_put_handler,
        .user_ctx = (void*) &ctx->pid_params,
    };
    httpd_register_uri_handler(server, &pid_uri);

    while(true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    httpd_stop(server);
    vTaskDelete(NULL);
}