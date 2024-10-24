#include "euler_pid_control.h"

#include <string.h>

#include "esp_log.h"

static const char* TAG = "EULER-PID";

static void calc_coeficients(euler_pid_params_t params, euler_pid_coeffs_t* coeffs) {
    coeffs->b0 = params.kp + params.ki + params.kd; // kp + ki + kd
    coeffs->b1 = -params.kp + params.ki - 2*params.kd; // -kp + ki - 2*kd
    coeffs->b2 = params.kd; // kd
}

static float cur_err_calc(double setpoint, double sample) {
    return setpoint - sample;
}

static double pid_calc(euler_pid_coeffs_t* pid_coeffs, double sample) {
    double output = 0.0;

    /* calculate current error */
    pid_coeffs->err_0 = cur_err_calc(pid_coeffs->setpoint, sample);

    /* calculate output by 2P2Z PID */
    /* U(z) = E(z) * b0 + E(z) * b1 * z^-1 + E(z) * b2 * z^-2 - U(z) * a1 * z^-1 */
    output = (pid_coeffs->b0 * pid_coeffs->err_0) 
            + (pid_coeffs->b1 * pid_coeffs->err_1) 
            + (pid_coeffs->b2 * pid_coeffs->err_2) 
            + (- pid_coeffs->out_1);

    /* anti-windup */
    output = (output > pid_coeffs->max_output) ? pid_coeffs->max_output : output;
    output = (output < - pid_coeffs->max_output) ? - pid_coeffs->max_output : output;

    /* update variables */
    pid_coeffs->err_2 = pid_coeffs->err_1;
    pid_coeffs->err_1 = pid_coeffs->err_0;
    pid_coeffs->out_1 = output;
    
    return output;
}

esp_err_t euler_pid_init(euler_pid_control_t* pid, euler_pid_params_t* params) {
    if(!pid && !params) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    bzero(&pid->_coeffs, sizeof(euler_pid_coeffs_t));
    calc_coeficients(*params, &pid->_coeffs);
    pid->_coeffs.setpoint = params->setpoint;
    pid->_coeffs.max_output = params->max_output;

    return ESP_OK;
}

esp_err_t euler_pid_compute(euler_pid_control_t* pid, double sample, double* output) {
    if(!pid || !output) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    *output = pid_calc(&pid->_coeffs, sample);

    return ESP_OK;
}

esp_err_t euler_pid_update_config(euler_pid_control_t* pid, euler_pid_params_t* params) {
    return euler_pid_init(pid, params);
}