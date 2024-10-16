#ifndef PWM_HELPER_H
#define PWM_HELPER_H

#include <stdint.h>

uint32_t map_rpm2pwm(float val, float min, float max);

#endif