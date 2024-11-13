#include "euler_restful_server.h"
#include "euler_helper.h"
#include "euler_macros.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

#include "cJSON.h"

extern QueueHandle_t update_ctrl_queue;

static const char* TAG = "restful";

static void add_cors_headers(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

/* GET */
esp_err_t root_get_handler(httpd_req_t* req) {
    add_cors_headers(req);

    const char* resp = "success";
    httpd_resp_send(req, resp, strlen(resp));

    return ESP_OK;
}

esp_err_t monitor_get_handler(httpd_req_t* req) {
    add_cors_headers(req);

    euler_monitor_t* monitor = (euler_monitor_t*) req->user_ctx;

    cJSON* json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "kp", monitor->pid->_kp);
    cJSON_AddNumberToObject(json, "ki", monitor->pid->_ki);
    cJSON_AddNumberToObject(json, "kd", monitor->pid->_kd);
    cJSON_AddNumberToObject(json, "setpoint", monitor->pid->_setpoint);
    cJSON_AddNumberToObject(json, "max_output", monitor->pid->_max_output);
    cJSON_AddNumberToObject(json, "min_output", monitor->pid->_min_output);
    cJSON_AddNumberToObject(json, "pulses", monitor->pulses);
    cJSON_AddNumberToObject(json, "duty_cycle", monitor->duty_cycle);
    cJSON_AddNumberToObject(json, "rpm", monitor->rpm);

    const char* resp = cJSON_Print(json);
    if(!resp) {
        ESP_LOGE(TAG, "failed to print json");
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    } else {
        httpd_resp_send(req, resp, strlen(resp));
    }

    cJSON_Delete(json);
    free((void*) resp);
    return ESP_OK;
}

/* PUT */
esp_err_t pid_put_handler(httpd_req_t* req) {
    add_cors_headers(req);

    euler_pid_params_t* pid_params = (euler_pid_params_t*) req->user_ctx;
    bool updated_flag = false;

    char buf[512] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if(ret <= 0) {
        if(ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    cJSON* json = cJSON_Parse(buf);
    if(!json) {
        ESP_LOGE(TAG, "failed to parse json");
        return ESP_FAIL;
    }

    if(cJSON_HasObjectItem(json, "kp")) {
        cJSON* kp = cJSON_GetObjectItem(json, "kp");
        if(cJSON_IsNumber(kp)) {
            updated_flag = true;
            pid_params->kp = kp->valuedouble;
            ESP_LOGI(TAG, "updated kp: %f", kp->valuedouble);
        }
    }
    if(cJSON_HasObjectItem(json, "ki")) {
        cJSON* ki = cJSON_GetObjectItem(json, "ki");
        if(cJSON_IsNumber(ki)) {
            updated_flag = true;
            pid_params->ki = ki->valuedouble;
            ESP_LOGI(TAG, "updated ki: %f", ki->valuedouble);
        }
    }
    if(cJSON_HasObjectItem(json, "kd")) {
        cJSON* kd = cJSON_GetObjectItem(json, "kd");
        if(cJSON_IsNumber(kd)) {
            updated_flag = true;
            pid_params->kd = kd->valuedouble;
            ESP_LOGI(TAG, "updated kd: %f", kd->valuedouble);
        }
    }
    if(cJSON_HasObjectItem(json, "setpoint")) {
        cJSON* setpoint = cJSON_GetObjectItem(json, "setpoint");
        if(cJSON_IsNumber(setpoint)) {
            updated_flag = true;
            pid_params->setpoint = setpoint->valuedouble;
            ESP_LOGI(TAG, "updated setpoint: %f", setpoint->valuedouble);
        }
    }
    if(cJSON_HasObjectItem(json, "max_output")) {
        cJSON* max_output = cJSON_GetObjectItem(json, "max_output");
        if(cJSON_IsNumber(max_output)) {
            updated_flag = true;
            pid_params->max_output = max_output->valuedouble;
            ESP_LOGI(TAG, "updated max_output: %f", max_output->valuedouble);
        }
    }

    if(updated_flag) {
        xQueueSend(update_ctrl_queue, pid_params, 0);
        ESP_ERROR_CHECK(nvs_set_pid_params(PID_NVS_PARAMS, pid_params));
    }

    cJSON_Delete(json);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}