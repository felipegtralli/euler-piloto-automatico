#ifndef PWM_HELPER_H
#define PWM_HELPER_H

#include <stdint.h>

double constrain(double x, double min, double max);
double map_2pwm(double val, double min, double max);

#endif