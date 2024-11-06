#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"

#include "euler_macros.h"
#include "euler_filter.h"
#include "euler_pid_control.h"
#include "euler_wifi.h"
#include "euler_tasks.h"
#include "euler_nvs_helper.h"
#include "euler_bdc_motor.h"
#include "euler_server.h"

/* GLOBALS */
static const char* TAG = "EULER-MAIN";

TaskHandle_t ctrl_task_handle;
QueueHandle_t update_ctrl_queue;
QueueHandle_t monitor_queue;

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
    static ctrl_loop_context_t ctrl_ctx = {0};

    /* define server comm context */
    static tcp_server_context_t server_ctx = {0};

    /* initialize pid control */
    /* read params from nvs */
    euler_pid_params_t pid_params = {0};
    ESP_ERROR_CHECK(nvs_get_pid_params(PID_NVS_PARAMS, &pid_params));

    static euler_pid_control_t pid;
    ESP_ERROR_CHECK(euler_pid_init(&pid, &pid_params));
    ctrl_ctx.pid = &pid;
    ctrl_ctx.pid_params = pid_params;
    server_ctx.pid_params = pid_params;

    /* initialize ema filter */
    /* read params from nvs */
    euler_filter_config_t filter_config = {0};
    ESP_ERROR_CHECK(nvs_get_filter_config(FILTER_NVS_CONFIG, &filter_config));

    static euler_filter_t filter = {0};
    ESP_ERROR_CHECK(euler_filter_init(&filter, &filter_config));
    ctrl_ctx.filter = &filter;

    /* initialize motor */
    static euler_bdc_motor_t motor = {
        .pwma = BDC_MOTOR_PWMA,
        .pwmb = BDC_MOTOR_PWMB,
        .unit = MCPWM_UNIT_0,
        .timer = MCPWM_TIMER_0,
    };
    ESP_ERROR_CHECK(euler_bdc_motor_gpio_init(&motor));

    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(motor.unit, motor.timer, &pwm_config));
    ctrl_ctx.motor = &motor;

    /* initialize encoder */
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_GPIO,    
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, 
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,    
        .lctrl_mode = PCNT_MODE_KEEP,    
        .hctrl_mode = PCNT_MODE_KEEP,   
        .counter_h_lim = ENCODER_HIGH_LIMIT,         
        .counter_l_lim = ENCODER_LOW_LIMIT, 
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    ctrl_ctx.encoder_unit = PCNT_UNIT_0;

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
        .alarm_count = (1.0 / SAMPLING_INTERVAL_HZ) * 1000000, // alarm every 1.667ms
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

    /* initialize monitor queue */
    monitor_queue = xQueueCreate(5, sizeof(euler_monitor_t));
    if(!monitor_queue) {
        ESP_LOGE(TAG, "failed to create monitor queue");
        return;
    }

    /* start tcp server task */
    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, &server_ctx, 1, NULL);
}