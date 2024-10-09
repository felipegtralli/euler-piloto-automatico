#include "euler_sensor.h"

#include <stdio.h>

double euler_read_sensor() {
    return (float) rand() / 100000;
}