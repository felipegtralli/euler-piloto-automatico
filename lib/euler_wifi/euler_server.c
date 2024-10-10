#include "euler_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "euler_macros.h"
#include "messages_helper.h"
#include "euler_nvs_helper.h"

#define MAX_BUFFER 128

extern QueueHandle_t update_ctrl_queue;

static const char* TAG = "EULER-SERVER";

static bool parse_args(int sock, char* data, size_t len, char args[2][32], uint8_t* argc) {
    uint8_t argi = 0;

    if(data[len - 1] == '\n') {
        data[len - 1] = '\0';
        --len;
    }

    for(int i = 0; i < len; i++) {
        if(data[i] == ' ') {
            args[*argc][argi] = '\0';
            argi = 0;

            if(++(*argc) >= 2) {
                write_message(sock, TOO_MANY_ARGS);
                ESP_LOGE(TAG, "too many arguments");
                return false;
            }
        } else {
            args[*argc][argi++] = data[i];

            if(argi >= 32) {
                write_message(sock, ARG_TOO_LONG);
                ESP_LOGE(TAG, "argument too long");
                return false;
            }
        }
    }
    args[*argc][argi] = '\0';

    return true;
}

bool is_valid_float(const char* str) {
    char* endptr;
    strtof(str, &endptr);
    if(endptr == str || *endptr != '\0') {
        return false;
    }
    return true;
}

static void handle_command(int sock, char args[2][32], euler_pid_params_t* pid_params) {
    if(args[0][0] == '\0') {
        write_message(sock, NO_ARGS);
        ESP_LOGE(TAG, "no arguments");
        return;
    }

    bool valid_command = true;
    if(strcmp(args[0], "help") == 0) {
        write_message(sock, HELP);
        return;
    } else if(args[1][0] == '\0') {
        write_message(sock, MISSING_ARG);
        return;
    } else if(!is_valid_float(args[1])) {
        write_message(sock, INVALID_ARG);
        ESP_LOGE(TAG, "invalid float argument: %s", args[1]);
        return;
    } else {
        switch(args[0][0]) {
            case 'k':
                if(strcmp(args[0], "kp") == 0) {
                    pid_params->kp = strtof(args[1], NULL);
                    write_message(sock, KP);
                } else if(strcmp(args[0], "ki") == 0) {
                    pid_params->ki = strtof(args[1], NULL);
                    write_message(sock, KI);
                } else if(strcmp(args[0], "kd") == 0) {
                    pid_params->kd = strtof(args[1], NULL);
                    write_message(sock, KD);
                } else {
                    valid_command = false;
                }
                break;
            case 's':
                if(strcmp(args[0], "setpoint") == 0) {
                    pid_params->setpoint = strtof(args[1], NULL);
                    write_message(sock, SETPOINT);
                } else {
                    valid_command = false;
                }
                break;
            case 'm':
                if(strcmp(args[0], "max_output") == 0) {
                    pid_params->max_output = strtof(args[1], NULL);
                    write_message(sock, MAX_OUTPUT);
                } else {
                    valid_command = false;
                }
                break;
            default:
                valid_command = false;
        }
    }
    if(valid_command) {
        xQueueSend(update_ctrl_queue, pid_params, 0);
        ESP_ERROR_CHECK(nvs_set_pid_params(PID_NVS_PARAMS, pid_params));
    } else {
        write_message(sock, UNKNOWN_COMMAND);
        ESP_LOGE(TAG, "unknown command: %s", args[0]);
    }
}

static void handle_data(int sock, char* data, size_t len, euler_pid_params_t* pid_params) {
    char args[2][32] = {{0}};
    uint8_t argc = 0;

    if(!parse_args(sock, data, len, args, &argc)) {
        return;
    }
    handle_command(sock, args, pid_params);
}

bool euler_receive_data(int sock, euler_pid_params_t* pid_params) {
    char buffer[MAX_BUFFER] = {0};
    char data[MAX_BUFFER] = {0};

    int len = read(sock, buffer, sizeof(buffer));
    if(len > 0) {
        buffer[len] = 0;

        if(strlen(data) + strlen(buffer) >= sizeof(data)) {
            ESP_LOGE(TAG, "buffer overflow...");
            return false;
        } else {
            strncat(data, buffer, sizeof(data) - strlen(data) - 1);
        }

        handle_data(sock, data, strlen(data), pid_params);
    } else if(len == 0) {
        ESP_LOGW(TAG, "closed connection");
        return false;
    } else {
        ESP_LOGE(TAG, "failed to read data");
        return false;
    }
    return true;
}