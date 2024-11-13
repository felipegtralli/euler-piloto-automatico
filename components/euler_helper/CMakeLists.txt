idf_component_register(SRCS "euler_helper.c"
                    INCLUDE_DIRS "include"
                    REQUIRES euler_macros euler_filter euler_pid_control nvs_flash esp_wifi)
