#ifndef EULER_FILTER_H
#define EULER_FILTER_H

#include "esp_err.h"

/**
 * @brief Filter parameters
 * 
 */
typedef struct {
    float fs; // sampling frequency
    float cutoff; // cutoff frequency
} euler_filter_params_t;

/**
 * @brief Filter coefficients
 * 
 */
typedef struct {
    float b0; // b0
    float b1; // b1
    float b2; // b2
    float a1; // a1
    float a2; // a2
    float x0; // sample 0
    float x1; // sample 1
    float x2; // sample 2
    float y0; // output 0
    float y1; // output 1
    float y2; // output 2
} euler_filter_coeffs_t;

/**
 * @brief Filter structure
 * 
 */
typedef struct {
    euler_filter_coeffs_t _coeffs; // filter coefficients
} euler_filter_t;

/**
 * @brief Initialize filter
 * 
 * @param[in] filter euler filter structure
 * @param[in] params filter parameters
 * @return 
 *   - ESP_OK: euler filter initialized successfully
 *   - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_filter_init(euler_filter_t* filter, euler_filter_params_t* params);

/**
 * @brief Apply butterworth 2nd order low pass filter
 * 
 * @param[in] filter euler filter structure
 * @param[in] sample current sample
 * @param[out] sample filtered sample
 * @return 
 *   - ESP_OK: butterworth filter applied successfully
 *   - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_filter_butter_2order_low(euler_filter_t* filter, float* sample);

/**
 * @brief Update euler filter parameters
 * 
 * @param[in] filter euler filter structure
 * @param[in] params filter parameters
 * @return 
 *   - ESP_OK: euler filter updated successfully
 *   - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_filter_update(euler_filter_t* filter, euler_filter_params_t* params);

#endif     // EULER_FILTER.H