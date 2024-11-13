idf_component_register(SRCS "euler_tasks.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver euler_pid_control euler_filter euler_bdc_motor euler_sensor euler_helper euler_wifi_ap euler_restful_server)
