#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gptimer.h"

#include "euler_macros.h"
#include "euler_filter.h"
#include "euler_pid_control.h"
#include "euler_wifi.h"
#include "euler_tasks.h"
#include "euler_nvs_helper.h"

/* DEFINES */
#define SAMPLING_INTERVAL_MS 100 

/* GLOBALS */
static const char* TAG = "EULER-MAIN";

TaskHandle_t ctrl_task_handle;
QueueHandle_t update_ctrl_queue;

void app_main(void) {
    /**** SETUP ****/

    /* initialize nvs flash */
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* define control loop context */
    static ctrl_loop_context_t ctrl_ctx;

    /* define server comm context */
    static tcp_server_context_t server_ctx;

    /* initialize pid control */
    /* read params from nvs */
    euler_pid_params_t pid_params = {0};
    ESP_ERROR_CHECK(nvs_get_pid_params(PID_NVS_PARAMS, &pid_params));

    static euler_pid_control_t pid;
    ESP_ERROR_CHECK(euler_pid_init(&pid, &pid_params));
    ctrl_ctx.pid = pid;
    ctrl_ctx.pid_params = pid_params;
    server_ctx.pid_params = pid_params;

    /* initialize butterworth filter */
    /* read params from nvs */
    euler_filter_params_t filter_params = {0};
    ESP_ERROR_CHECK(nvs_get_filter_params(FILTER_NVS_PARAMS, &filter_params));

    static euler_filter_t filter;
    ESP_ERROR_CHECK(euler_filter_init(&filter, &filter_params));
    ctrl_ctx.filter = filter;

    /* initialize wifi access point */
    euler_wifi_init_ap();

    /* initialize update pid control queue */
    update_ctrl_queue = xQueueCreate(5, sizeof(euler_pid_params_t));
    if(!update_ctrl_queue) {
        ESP_LOGE(TAG, "failed to create update control queue");
        return;
    }

    /* create control task (loop) */
    ESP_LOGI(TAG, "creating control task");
    xTaskCreate(ctrl_task, "main_ctrl_task", 4096, &ctrl_ctx, 2, &ctrl_task_handle);

    /* create control loop hardware timer */
    gptimer_handle_t timer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 tick per us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    /* configure loop to SAMPLING_INTERVAL_MS */
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = SAMPLING_INTERVAL_MS * 1000, // alarm every 100ms
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

    /* register isr callback */
    gptimer_event_callbacks_t timer_callbacks = {
        .on_alarm = ctrl_task_timer_handle, // callback isr function
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &timer_callbacks, NULL));

    /* start control loop timer */
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));

    /* start tcp server task */
    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, &server_ctx, 1, NULL);
}