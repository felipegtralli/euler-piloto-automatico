#ifndef BDC_MOTOR_H
#define BDC_MOTOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_attr.h"
#include "esp_err.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

/**
 * @brief BDC motor structure
 * 
 */
typedef struct {
    int pwma;
    int pwmb;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
} euler_bdc_motor_t;

/**
 * @brief Initialize BDC motor GPIO
 * 
 * @param motor BDC motor structure
 */
esp_err_t euler_bdc_motor_gpio_init(euler_bdc_motor_t* motor);

/**
 * @brief Set BDC motor speed
 * 
 * @param motor BDC motor structure
 * @param duty_cycle Duty cycle
 */
esp_err_t euler_bdc_motor_set_speed(euler_bdc_motor_t* motor, float duty_cycle);

#endif