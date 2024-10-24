#include "euler_sensor.h"

#include "euler_macros.h"

#include "driver/pulse_cnt.h"

double euler_read_sensor() {
    return (float) (rand() % 6800);
}

double euler_pulses2rpm(int pulses) {
    double rev =(double) pulses / ENCODER_PULSES_PER_REV;
    
    return (rev / (1.0 / SAMPLING_INTERVAL_HZ)) * 60;
}