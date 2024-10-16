#include "euler_pwm_helper.h"

#include "euler_macros.h"

static float constrain(float x, float min, float max) {
    if(x > max) {
        return max;
    } else if(x < min) {
        return min;
    } else {
        return x;
    }
}

uint32_t map_rpm2pwm(float val, float min, float max) {
    val = constrain(val, min, max);
    float d = (val / max) * 100;

    return (d / 100) * BDC_MCPWM_DUTY_TICK_MAX;
}   