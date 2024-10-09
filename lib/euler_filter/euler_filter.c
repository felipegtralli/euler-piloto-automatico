#include "euler_filter.h"

#include <math.h>
#include <strings.h>

#include "esp_log.h"

static const char* TAG = "EULER-FILTER";

static void calc_coeficients(euler_filter_params_t params, euler_filter_coeffs_t* filter_coeffs) {
    /* frequency ratio */
    double ff = params.cutoff / params.fs;

    const double ita = 1.0 / tan(M_PI * ff);
    const double q = sqrt(2.0);

    /* coeficients */ 
    filter_coeffs->b0 = 1.0 / (1.0 + q*ita + ita * ita);
    filter_coeffs->b1 = 2 * filter_coeffs->b0;
    filter_coeffs->b2 = filter_coeffs->b0;
    filter_coeffs->a1 = 2.0 * (ita * ita - 1.0) * filter_coeffs->b0;
    filter_coeffs->a2 = - (1.0 - q * ita + ita * ita) * filter_coeffs->b0;
}

static float calc_butter_2order_low(euler_filter_coeffs_t* filter_coeffs, float sample) {
    /* shifiting samples */
    filter_coeffs->x2 = filter_coeffs->x1;
    filter_coeffs->x1 = filter_coeffs->x0;
    filter_coeffs->x0 = sample;

    /* calculates output */
    /* Y(z) = b0 * X(z) + b1 * X(z^-1) + b2 * X(z^-2) - a1 * Y(z^-1) - a2 * Y(z^-2) */
    filter_coeffs->y0 = filter_coeffs->b0 * filter_coeffs->x0
                + filter_coeffs->b1 * filter_coeffs->x1
                + filter_coeffs->b2 * filter_coeffs->x2
                - filter_coeffs->a1 * filter_coeffs->y1 
                - filter_coeffs->a2 * filter_coeffs->y2;
    
    /* shifiting output */
    filter_coeffs->y2 = filter_coeffs->y1;
    filter_coeffs->y1 = filter_coeffs->y0;

    return filter_coeffs->y0;
}

esp_err_t euler_filter_init(euler_filter_t* filter, euler_filter_params_t* params) {
    if(!filter) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    bzero(&filter->_coeffs, sizeof(euler_filter_coeffs_t));
    calc_coeficients(*params, &filter->_coeffs);

    return ESP_OK;
}

esp_err_t euler_filter_butter_2order_low(euler_filter_t* filter, float* sample) {
    if(!filter || !sample) {
        ESP_LOGE(TAG, "invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    *sample = calc_butter_2order_low(&filter->_coeffs, *sample);

    return ESP_OK;
}

esp_err_t euler_filter_update(euler_filter_t* filter, euler_filter_params_t* params) {
    return euler_filter_init(filter, params);
}