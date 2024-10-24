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

/* ENCODER */
#define ENCODER_GPIO 34
#define ENCODER_HIGH_LIMIT 3000
#define ENCODER_LOW_LIMIT -3000
#define ENCODER_PULSES_PER_REV 20

/* PWM */
#define BDC_MOTOR_PWMA 5
#define BDC_MOTOR_PWMB 18
#define RPM 7000
#define PWM_FREQ (RPM / ENCODER_PULSES_PER_REV)

#endif