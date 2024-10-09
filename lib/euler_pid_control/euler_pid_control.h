#ifndef EULER_PID_CONTROL_H
#define EULER_PID_CONTROL_H

#include "esp_err.h"

/**
 * @brief PID control configuration
 * 
 */
typedef struct {
    float kp; // kp
    float ki; // ki
    float kd; // kd
    float setpoint; // setpoint
    float max_output; // max output (anti-windup)
} euler_pid_params_t;

/**
 * @brief PID control coefficients
 * 
 */
typedef struct {
    float b0; // kp + ki + kd
    float b1; // -kp + ki - 2*kd
    float b2; // kd
    float err_0; // error (z)
    float err_1; // error 1 (z^-1)
    float err_2; // error 2 (z^-2)
    float out_1; // output (z^-1)
    float setpoint; // setpoint
    float max_output; // max output (anti-windup)
} euler_pid_coeffs_t;

/**
 * @brief PID control structure
 * 
 */
typedef struct {
    euler_pid_coeffs_t _coeffs; // PID coefficients
} euler_pid_control_t;

/**
 * @brief Initialize PID control structure
 * 
 * @param[in] pid PID control structure
 * @param[in] params PID control parameters
 * @return 
 *  - ESP_OK: PID control structure initialized successfully
 *  - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_pid_init(euler_pid_control_t* pid, euler_pid_params_t* params);

/**
 * @brief get PID control output
 * 
 * @param[in] pid PID control structure
 * @param[in] sample current sample
 * @param[out] output output value
 * @return 
 *  - ESP_OK: PID compute successfully
 *  - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_pid_compute(euler_pid_control_t* pid, float sample, float* output);

/**
 * @brief update PID control configuration
 * 
 * @param[in] pid PID control structure
 * @param[in] params PID control parameters
 * @return 
 *  - ESP_OK: PID control configuration updated successfully
 *  - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_pid_update_config(euler_pid_control_t* pid, euler_pid_params_t* params);

#endif  // EULER_PID_CONTROL_H