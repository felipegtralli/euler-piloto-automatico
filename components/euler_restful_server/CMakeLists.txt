idf_component_register(SRCS "euler_restful_server.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_server euler_pid_control euler_helper euler_macros json)
