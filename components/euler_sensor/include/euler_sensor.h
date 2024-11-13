#pragma once

#include "esp_err.h"

#include "driver/pcnt.h"

/**
 * @brief initialize encoder
 * 
 * @param pcnt_unit_t unit
 */
int16_t euler_encoder_get_pulses(pcnt_unit_t unit);

/**
 * @brief convert encoder pulses to RPM
 * 
 * @param double pulses
 */
double euler_pulses2rpm(double pulses);