#include "euler_bdc_motor.h"

#include "esp_log.h"

const char* TAG = "BDC-MOTOR";

esp_err_t euler_bdc_motor_gpio_init(euler_bdc_motor_t* motor) {
    ESP_LOGI(TAG, "initializing bdc motor");
    ESP_ERROR_CHECK(mcpwm_gpio_init(motor->unit, MCPWM0A, motor->pwma));
    ESP_ERROR_CHECK(mcpwm_gpio_init(motor->unit, MCPWM0B, motor->pwmb));

    return ESP_OK;
}

esp_err_t euler_bdc_motor_set_speed(euler_bdc_motor_t* motor, float duty_cycle) {
    if(duty_cycle < -100.0 || duty_cycle > 100.0) {
        ESP_LOGE(TAG, "duty cycle out of range");
        return ESP_ERR_INVALID_ARG;
    }

    if(duty_cycle > 0.0) { // forward
        ESP_ERROR_CHECK(mcpwm_set_signal_low(motor->unit, motor->timer, MCPWM_OPR_B));
        ESP_ERROR_CHECK(mcpwm_set_duty(motor->unit, motor->timer, MCPWM_OPR_A, duty_cycle));
        ESP_ERROR_CHECK(mcpwm_set_duty_type(motor->unit, motor->timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0));
    } else if(duty_cycle < 0.0) { // backward
        ESP_ERROR_CHECK(mcpwm_set_signal_low(motor->unit, motor->timer, MCPWM_OPR_A));
        ESP_ERROR_CHECK(mcpwm_set_duty(motor->unit, motor->timer, MCPWM_OPR_B, -duty_cycle));
        ESP_ERROR_CHECK(mcpwm_set_duty_type(motor->unit, motor->timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0));
    } else { // stop
        ESP_ERROR_CHECK(mcpwm_set_signal_low(motor->unit, motor->timer, MCPWM_OPR_A));
        ESP_ERROR_CHECK(mcpwm_set_signal_low(motor->unit, motor->timer, MCPWM_OPR_B));
    }
    
    return ESP_OK;
}