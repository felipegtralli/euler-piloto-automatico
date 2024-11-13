#pragma once

#include "euler_pid_control.h"
#include "euler_filter.h"
#include "euler_bdc_motor.h"
#include "euler_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"

/**
 * @brief Control loop context
 * 
 */
typedef struct {
    euler_pid_control_t* pid;
    euler_pid_params_t pid_params;
    euler_filter_t* filter;
    euler_bdc_motor_t* motor;
    pcnt_unit_t encoder_unit;
} ctrl_loop_context_t;

/**
 * @brief Restful server context
 */
typedef struct {
    euler_pid_params_t pid_params;
} restful_server_context_t;

/**
 * @brief Monitor flag type
 * 
 */
typedef bool monitor_flag_t;

/**
 * @brief Control loop task
 * 
 * @param pvParameters control loop context
 */
void ctrl_task(void* pvParameters);

/**
 * @brief Timer callback to notify control task from isr
 * 
 */
bool ctrl_task_timer_handle(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

/**
 * @brief Restful server task
 * 
 * @param pvParameters restful server context
 */
void restful_server_task(void* pvParameters);