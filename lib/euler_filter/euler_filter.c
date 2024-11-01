#include "euler_filter.h"

#include <math.h>
#include <strings.h>

#include "esp_log.h"

static const char* TAG = "EULER-FILTER";

static double calc_alpha(size_t n) {
    return 2.0 / (n + 1);
}

static double calc_exp_mov_avg(euler_filter_t* filter, double sample) {
    double new_avg = filter->_alpha * sample + (1 - filter->_alpha) * filter->_cur_value;
    
    if(new_avg > filter->_max) {
        return filter->_max;
    } else if(new_avg < filter->_min) {
        return filter->_min;
    } else {
        return new_avg;
    }
}

esp_err_t euler_filter_init(euler_filter_t* filter, euler_filter_config_t* config) {
    if(!filter || !config) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    filter->_alpha = calc_alpha(config->n);
    filter->_cur_value = 0.0;
    filter->_max = config->max;
    filter->_min = config->min;

    return ESP_OK;
}

esp_err_t euler_filter_exp_mov_avg(euler_filter_t* filter, double* sample) {
    if(!filter) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    filter->_cur_value = calc_exp_mov_avg(filter, *sample);
    *sample = filter->_cur_value;

    return ESP_OK;
}

esp_err_t euler_filter_update(euler_filter_t* filter, euler_filter_config_t* config) {
    return euler_filter_init(filter, config);
}