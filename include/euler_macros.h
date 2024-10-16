#ifndef MACROS_H
#define MACROS_H

/* ZOH */
#define SAMPLING_INTERVAL_HZ 600 

/* NVS */
#define PID_NVS_STORAGE "pid_storage"
#define PID_NVS_PARAMS "pid_params"
#define FILTER_NVS_STORAGE "filter_storage"
#define FILTER_NVS_PARAMS "filter_params"
#define WIFI_NVS_STORAGE "wifi_storage"
#define WIFI_NVS_CONFIG "wifi_config"

/* PWM */
#define BDC_MOTOR_MCPWM_GPIOA 5
#define BDC_MOTOR_MCPWM_GPIOB 18
#define BDC_MCPWM_TIMER_RES 1000000 // 1us
#define BDC_MCPWM_TIMER_FREQ 25000 // 25kHz
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RES / BDC_MCPWM_TIMER_FREQ)

/* ENCODER */
#define ENCODER_GPIO 34
#define ENCODER_HIGH_LIMIT 3000
#define ENCODER_LOW_LIMIT -3000
#define ENCODER_PULSES_PER_REV 20

#endif