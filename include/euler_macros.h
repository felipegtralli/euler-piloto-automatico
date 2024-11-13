#ifndef MACROS_H
#define MACROS_H

/* ZOH */
#define SAMPLING_INTERVAL_HZ 30

/* NVS */
#define PID_NVS_STORAGE "pid_storage"
#define PID_NVS_PARAMS "pid_params"
#define FILTER_NVS_STORAGE "filter_storage"
#define FILTER_NVS_CONFIG "filter_config"
#define WIFI_NVS_STORAGE "wifi_storage"
#define WIFI_NVS_CONFIG "wifi_config"

/* ENCODER */
#define ENCODER_GPIO 19
#define ENCODER_HIGH_LIMIT 80
#define ENCODER_LOW_LIMIT 0
#define ENCODER_PULSES_PER_REV 5

/* PWM */
#define BDC_MOTOR_PWMA 5
#define BDC_MOTOR_PWMB 18
#define PWM_FREQ 20000 // 20kHz

/* LED */
#define LED1_PIN 0

#endif