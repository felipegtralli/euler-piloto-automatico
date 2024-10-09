#ifndef EULER_SERVER_H
#define EULER_SERVER_H

#include <string.h>
#include <sys/param.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "euler_pid_control.h"

#define PORT 8080

/**
 * @brief Receive data from tcp client
 */
bool euler_receive_data(int sock, euler_pid_params_t* pid_params);

#endif    // EULER_SERVER.H