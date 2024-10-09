#include "messages_helper.h"

#include <string.h>
#include <unistd.h>

/* normal messages */
const char* help_message = "EULER-PILOTO-AUTOMATICO\n"
                           "-----------------------\n\n"
                           "Commands:\n"
                           "    help                show help message\n"
                           "    kp <value>          update kp value\n"
                           "    ki <value>          update ki value\n"
                           "    kd <value>          update kd value\n"
                           "    setpoint <value>    update setpoint value\n"
                           "    max_output <value>  update max output value\n";

const char* update_kp = "Updated kp value\n";
const char* update_ki = "Updated ki value\n";
const char* update_kd = "Updated kd value\n";
const char* update_setpoint = "Updated setpoint value\n";
const char* update_max_output = "Updated max output value\n";

/* error messages */
const char* unknown_message = "ERROR: unknown command\n"
                              "type 'help' for available commands\n";
const char* too_many_args = "ERROR: too many arguments\n";
const char* arg_too_long = "ERROR: argument too long\n";
const char* no_args = "ERROR: no arguments\n";
const char* missing_arg = "ERROR: missing argument\n";
const char* invalid_arg = "ERROR: invalid argument\n";


void write_message(int sock, int8_t code) {
    switch(code) {
        case HELP:
            write(sock, help_message, strlen(help_message));
            break;
        case KP:
            write(sock, update_kp, strlen(update_kp));
            break;
        case KI:
            write(sock, update_ki, strlen(update_ki));
            break;
        case KD:
            write(sock, update_kd, strlen(update_kd));
            break;
        case SETPOINT:
            write(sock, update_setpoint, strlen(update_setpoint));
            break;
        case MAX_OUTPUT:
            write(sock, update_max_output, strlen(update_max_output));
            break;
        
        case TOO_MANY_ARGS:
            write(sock, too_many_args, strlen(too_many_args));
            break;
        case ARG_TOO_LONG:
            write(sock, arg_too_long, strlen(arg_too_long));
            break;
        case NO_ARGS:
            write(sock, no_args, strlen(no_args));
            break;
        case MISSING_ARG:
            write(sock, missing_arg, strlen(missing_arg));
            break;
        case INVALID_ARG:
            write(sock, invalid_arg, strlen(invalid_arg));
            break;
            
        default:
            write(sock, unknown_message, strlen(unknown_message));
            break;
    }
}