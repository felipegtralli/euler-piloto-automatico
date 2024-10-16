#include "euler_sensor.h"

#include "euler_macros.h"

#include "driver/pulse_cnt.h"

double euler_read_sensor() {
    long r = rand() % 7000;
    return (float) r;
}

float euler_pulses2rpm(int pulses) {
    float rev =(float) pulses / ENCODER_PULSES_PER_REV;
    
    return (rev / (1.0 / SAMPLING_INTERVAL_HZ)) * 60;
}