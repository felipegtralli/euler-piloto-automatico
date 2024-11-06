#ifndef MESSAGES_HELPER_H
#define MESSAGES_HELPER_H

#include "freertos/FreeRTOS.h"

/* normal codes */
#define HELP 1
#define KP 2
#define KI 3
#define KD 4
#define SETPOINT 5
#define MAX_OUTPUT 6
#define MIN_OUTPUT 7

/* error codes */
#define UNKNOWN_COMMAND -1
#define TOO_MANY_ARGS -2
#define ARG_TOO_LONG -3
#define NO_ARGS -4
#define INVALID_ARG -5

void write_message(int sock, int8_t code);
void send_monitor_value(int sock, const char* label, float value);

#endif // MESSAGES_HELPER_H