#include "euler_sensor.h"
#include "euler_macros.h"

#include "driver/pcnt.h"
#include "esp_err.h"

int16_t euler_encoder_get_pulses(pcnt_unit_t unit) {
    int16_t count = 0;
    pcnt_get_counter_value(unit, &count);
    pcnt_counter_clear(unit);

    return count;
}

double euler_pulses2rpm(double pulses) {
    double rev = pulses / ENCODER_PULSES_PER_REV;
    
    return (rev / (1.0 / SAMPLING_INTERVAL_HZ)) * 60;
}