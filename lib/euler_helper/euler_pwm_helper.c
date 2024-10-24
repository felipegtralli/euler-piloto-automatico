#include "euler_pwm_helper.h"

#include "euler_macros.h"

static double constrain(double x, double min, double max) {
    if(x > max) {
        return max;
    } else if(x < min) {
        return min;
    } else {
        return x;
    }
}

double map_rpm2pwm(double val, double min, double max) {
    val = constrain(val, min, max);
    return (val / max) * 100;
}   