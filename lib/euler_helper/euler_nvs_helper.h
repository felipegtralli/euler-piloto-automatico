#ifndef EULER_NVS_HELPER_H
#define EULER_NVS_HELPER_H

#include "nvs_flash.h"

float nvs_get_float(nvs_handle_t handle, const char* key, float default_value);

#endif // EULER_NVS_HELPER_H