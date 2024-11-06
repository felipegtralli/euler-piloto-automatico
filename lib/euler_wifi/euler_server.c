#include "euler_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "euler_macros.h"
#include "messages_helper.h"
#include "euler_nvs_helper.h"
#include "euler_tasks.h"

#define MAX_BUFFER 128
#define MAX_MONITOR_DURATION_S 60

enum {kp, ki, kd, setpoint, max_out, min_out, out, pulses, duty_cycle, __padding, args_len};
#define _KP (1ULL << kp) // 1 
#define _KI (1ULL << ki) // 10
#define _KD (1ULL << kd) // 100
#define _SETPOINT (1ULL << setpoint) // 1000
#define _MAX_OUT (1ULL << max_out) // 10000
#define _MIN_OUT (1ULL << min_out) // 100000
#define _OUT (1ULL << out) // 1000000
#define _PULSES (1ULL << pulses) // 10000000
#define _DUTY_CYCLE (1ULL << duty_cycle) // 100000000

#define UNKNOWN_COMMAND_FLAG (1U << 0)
#define UPDATE_PID_FLAG (1U << 1) 

extern QueueHandle_t update_ctrl_queue;
extern QueueHandle_t monitor_queue;
extern monitor_flag_t monitor_flag;

static const char* TAG = "EULER-SERVER";
static euler_monitor_t monitor = {0};

bool is_valid_float(const char* str) {
    char* endptr;
    strtof(str, &endptr);
    if(endptr == str || *endptr != '\0') {
        return false;
    }
    return true;
}

static void handle_monitor(int sock, int argc, char* argv[]) {
    monitor_flag = true;
    unsigned long long var2monitor = 0;

    for(int i = 1; i < argc; i++) {
        if(strcmp(argv[i], "kp") == 0) {
            var2monitor |= _KP;
        } else if(strcmp(argv[i], "ki") == 0) {
            var2monitor |= _KI;
        } else if(strcmp(argv[i], "kd") == 0) {
            var2monitor |= _KD;
        } else if(strcmp(argv[i], "setpoint") == 0) {
            var2monitor |= _SETPOINT;
        } else if(strcmp(argv[i], "max_output") == 0) {
            var2monitor |= _MAX_OUT;
        } else if(strcmp(argv[i], "min_output") == 0) {
            var2monitor |= _MIN_OUT;
        } else if(strcmp(argv[i], "out") == 0) {
            var2monitor |= _OUT;
        } else if(strcmp(argv[i], "pulses") == 0) {
            var2monitor |= _PULSES;
        } else if(strcmp(argv[i], "duty_cycle") == 0) {
            var2monitor |= _DUTY_CYCLE;
        } else {
            write_message(sock, INVALID_ARG);
            ESP_LOGE(TAG, "invalid argument %s", argv[i]);
            return;
        }
    }

    fd_set read_fds;
    time_t start_time = time(NULL);
    while(monitor_flag && (time(NULL) - start_time < MAX_MONITOR_DURATION_S)) {
        FD_ZERO(&read_fds);
        FD_SET(sock, &read_fds);

        struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = 10000,
        };

        int ret = select(sock + 1, &read_fds, NULL, NULL, &timeout);
        if(ret > 0 && FD_ISSET(sock, &read_fds)) {
            char buffer[MAX_BUFFER] = {0};
            int len = read(sock, buffer, sizeof(buffer));
            if(len > 0) {
                buffer[len] = 0;
                if(strcmp(buffer, "stop") == 0) {
                    monitor_flag = false;
                    break;
                }
            } else {
                monitor_flag = false;
                ESP_LOGE(TAG, "failed to read data");
                break;
            }
        }

        xQueueReceive(monitor_queue, &monitor, portMAX_DELAY);
        if(var2monitor & _KP) {
            send_monitor_value(sock, "KP", monitor.pid->_kp);
        }
        if(var2monitor & _KI) {
            send_monitor_value(sock, "KI", monitor.pid->_ki);
        }
        if(var2monitor & _KD) {
            send_monitor_value(sock, "KD", monitor.pid->_kd);
        }
        if(var2monitor & _SETPOINT) {
            send_monitor_value(sock, "Setpoint", monitor.pid->_setpoint);
        }
        if(var2monitor & _MAX_OUT) {
            send_monitor_value(sock, "Max output", monitor.pid->_max_output);
        }
        if(var2monitor & _MIN_OUT) {
            send_monitor_value(sock, "Min output", monitor.pid->_min_output);
        }
        if(var2monitor & _OUT) {
            send_monitor_value(sock, "Output", monitor.pid->_last_output);
        }
        if(var2monitor & _PULSES) {
            send_monitor_value(sock, "Pulses", monitor.pulses);
        }
        if(var2monitor & _DUTY_CYCLE) {
            send_monitor_value(sock, "Duty cycle", monitor.duty_cycle);
        }
    }
    monitor_flag = false;
    write(sock, "monitor stopped", 16);
}

static bool parse_args(int sock, char* data, int data_len, int* argc, char* argv[]) {
    if(!data_len) {
        write_message(sock, NO_ARGS);
        ESP_LOGE(TAG, "no arguments");
        return false;
    }
    
    if(data[data_len - 1] == '\n') {
        data[data_len - 1] = '\0';
        --data_len;
    }

    char* token = strtok(data, " ");
    if(!token) {
        write_message(sock, NO_ARGS);
        ESP_LOGE(TAG, "no arguments");
        return false;
    }

    while(token) {
        if(*argc >= args_len) {
            write_message(sock, TOO_MANY_ARGS);
            ESP_LOGE(TAG, "too many arguments");
            return false;
        } 
        if(strlen(token) > 32) {
            write_message(sock, ARG_TOO_LONG);
            ESP_LOGE(TAG, "argument too long");
            return false;
        }
        argv[(*argc)++] = token;
        token = strtok(NULL, " ");
    }

    return true;
}

static void handle_command(int sock, int argc, char* argv[], euler_pid_params_t* pid_params) {
    uint8_t flags = 0;

    if(argc == 1) {
        if(strcmp(argv[0], "help") == 0) {
            write_message(sock, HELP);
            return;
        } else {
            flags |= UNKNOWN_COMMAND_FLAG;
        }
    } else if(argc > 1 && argc < args_len) {
        if(strcmp(argv[0], "monitor") == 0) {
            handle_monitor(sock, argc, argv);
        } else if(argc == 2) {
            if(!is_valid_float(argv[1])) {
                write_message(sock, INVALID_ARG);
                ESP_LOGE(TAG, "invalid float argument: %s", argv[1]);
                return;
            }

            flags |= UPDATE_PID_FLAG;
            switch(argv[0][0]) {

                case 'k':
                    if(strcmp(argv[0], "kp") == 0) {
                        pid_params->kp = strtof(argv[1], NULL);
                        write_message(sock, KP);
                    } else if(strcmp(argv[0], "ki") == 0) {
                        pid_params->ki = strtof(argv[1], NULL);
                        write_message(sock, KI);
                    } else if(strcmp(argv[0], "kd") == 0) {
                        pid_params->kd = strtof(argv[1], NULL);
                        write_message(sock, KD);
                    } else {
                        flags |= UNKNOWN_COMMAND_FLAG;
                    }
                    break;
                case 's':
                    if(strcmp(argv[0], "setpoint") == 0) {
                        pid_params->setpoint = strtof(argv[1], NULL);
                        write_message(sock, SETPOINT);
                    } else {
                        flags |= UNKNOWN_COMMAND_FLAG;
                    }
                    break;
                case 'm':
                    if(strcmp(argv[0], "max_output") == 0) {
                        pid_params->max_output = strtof(argv[1], NULL);
                        write_message(sock, MAX_OUTPUT);
                    } else if(strcmp(argv[0], "min_output") == 0) {
                        pid_params->min_output = strtof(argv[1], NULL);
                        write_message(sock, MIN_OUTPUT);
                    } else {
                        flags |= UNKNOWN_COMMAND_FLAG;
                    }
                    break;
                default:
                    flags |= UNKNOWN_COMMAND_FLAG;
                    flags &= ~UPDATE_PID_FLAG;
                    break;
            }
        } else {
            flags |= UNKNOWN_COMMAND_FLAG;
        }
    } else {
        flags |= UNKNOWN_COMMAND_FLAG;
    }

    if(flags & UNKNOWN_COMMAND_FLAG) {
        write_message(sock, UNKNOWN_COMMAND);
        ESP_LOGE(TAG, "unknown command: %s", argv[0]);
    }
    if(flags & UPDATE_PID_FLAG) {
        xQueueSend(update_ctrl_queue, pid_params, 0);
        ESP_ERROR_CHECK(nvs_set_pid_params(PID_NVS_PARAMS, pid_params));
    }
}

static void handle_data(int sock, char* data, int data_len, euler_pid_params_t* pid_params) {
    int argc = 0;
    char* argv[32] = {0};

    if(!parse_args(sock, data, data_len, &argc, argv)) {
        return;
    }
    
    handle_command(sock, argc, argv, pid_params);
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