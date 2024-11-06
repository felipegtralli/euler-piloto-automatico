#include "euler_pid_control.h"

#include <string.h>

#include "esp_log.h"

static const char* TAG = "EULER-PID";

static float cur_err_calc(double setpoint, double sample) {
    return setpoint - sample;
}

static double pid_calc_inc(euler_pid_control_t* pid, double sample) {
    double output = 0.0;

    /* calculate error */
    double err = cur_err_calc(pid->_setpoint, sample);

    /* calculate output by PID incremental */
    /* dU(z) = Kp * (1 - z^-1)E(z) + Ki * E(z) + Kd * (1 - 2z^-1 + z^-2)E(z) */
    output = pid->_kp * (err - pid->_prev_err1) + 
             pid->_ki * err +
             pid->_kd * (err - 2 * pid->_prev_err1 + pid->_prev_err2) +
             pid->_last_output;

    /* anti-windup */
    output = (output > pid->_max_output) ? pid->_max_output : output;
    output = (output < pid->_min_output) ? pid->_min_output : output;

    /* update errors */
    pid->_prev_err2 = pid->_prev_err1;
    pid->_prev_err1 = err;

    /* update last output */
    pid->_last_output = output;
    
    return output;
}

esp_err_t euler_pid_init(euler_pid_control_t* pid, euler_pid_params_t* params) {
    if(!pid && !params) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    pid->_kp = params->kp;
    pid->_ki = params->ki;
    pid->_kd = params->kd;
    pid->_setpoint = params->setpoint;
    pid->_max_output = params->max_output;
    pid->_prev_err1 = 0.0;
    pid->_prev_err2 = 0.0;
    pid->_last_output = 0.0;

    return ESP_OK;
}

esp_err_t euler_pid_compute(euler_pid_control_t* pid, double sample, double* output) {
    if(!pid || !output) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    *output = pid_calc_inc(pid, sample);

    return ESP_OK;
}

esp_err_t euler_pid_update_config(euler_pid_control_t* pid, euler_pid_params_t* params) {
    return euler_pid_init(pid, params);
}