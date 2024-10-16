#ifndef EULER_TASKS_H
#define EULER_TASKS_H

#include "euler_pid_control.h"
#include "euler_filter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"

/**
 * @brief Control loop context
 * 
 */
typedef struct {
    euler_pid_control_t pid;
    euler_pid_params_t pid_params;
    euler_filter_t filter;
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t encoder;
} ctrl_loop_context_t;

/**
 * @brief Server communication context
 * 
 */
typedef struct {
    euler_pid_params_t pid_params;
} tcp_server_context_t;

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
 * @brief Tcp server task
 * 
 * @param pvParameters server communication context
 */
void tcp_server_task(void* pvParameters);

#endif      // EULER_TASKS.H