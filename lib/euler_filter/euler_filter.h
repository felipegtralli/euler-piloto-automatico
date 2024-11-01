#ifndef EULER_FILTER_H
#define EULER_FILTER_H

#include "esp_err.h"

/**
 * @brief Filter configuration
 * 
 */
typedef struct {
    size_t n;
    double max;
    double min;
} euler_filter_config_t;

/**
 * @brief Filter structure
 * 
 */
typedef struct {
    double _alpha;
    double _cur_value;
    double _max;
    double _min;
} euler_filter_t;

/**
 * @brief Initialize filter
 * 
 * @param[in] filter euler filter structure
 * @param[in] config filter configuration
 * @return 
 *   - ESP_OK: euler filter initialized successfully
 *   - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_filter_init(euler_filter_t* filter, euler_filter_config_t* config);

/**
 * @brief Apply exponential moving average filter
 * 
 * @param[in] filter euler filter structure
 * @param[in] sample input sample
 * @return 
 *   - ESP_OK: euler filter applied successfully
 *   - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_filter_exp_mov_avg(euler_filter_t* filter, double* sample);

/**
 * @brief Update euler filter configuration
 * 
 * @param[in] filter euler filter structure
 * @param[in] config filter configuration
 * @return 
 *   - ESP_OK: euler filter updated successfully
 *   - ESP_ERR_INVALID_ARG: invalid arguments
 */
esp_err_t euler_filter_update(euler_filter_t* filter, euler_filter_config_t* config);

#endif     // EULER_FILTER.H