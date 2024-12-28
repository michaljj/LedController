#include <stdio.h>
#include "httpServerHandler.h"
#include <ctype.h>
#include <string.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include "mDNSHandler.h"
#include "mainStateHandler.h"
#include "nvsHandler.h"

static char *TAG = "HTTPSERVERHANDLER";
static httpd_handle_t httpServerHandle = NULL;

extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");

static void httpServerHandler_UrlDecode(const char *src, char *dest)
{
    while (*src)
    {
        if (*src == '%')
        {
            if (isxdigit((unsigned char)src[1]) && isxdigit((unsigned char)src[2]))
            {
                char hex[3] = {src[1], src[2], '\0'};
                *dest++ = (char)strtol(hex, NULL, 16);
                src += 3;
            }
            else
            {
                *dest++ = *src++;
            }
        }
        else if (*src == '+')
        {
            *dest++ = ' ';
            src++;
        }
        else
        {
            *dest++ = *src++;
        }
    }
    *dest = '\0';
}

static esp_err_t httpServerHandler_GetHandler(httpd_req_t *req)
{
    const uint32_t root_len = root_end - root_start;

    ESP_LOGI(TAG, "Serve root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, root_start, root_len);

    return ESP_OK;
}

static esp_err_t httpServerHandler_GetDataFromPost(char *const content, const char *const key, char *const val)
{
    int keyLength = strlen(key);
    if (0 != keyLength)
    {
        char *data = NULL;
        int contentLenght = strlen(content);
        char *contentLoc = malloc(sizeof(char) * (contentLenght + 1));
        strncpy(contentLoc, content, contentLenght);
        data = strtok(contentLoc, "=&");
        while (NULL != data)
        {
            if (0 == strcmp(data, key))
            {
                data = strtok(NULL, "=&");
                int len = strlen(data) + 1;
                strncpy(val, data, len);
                return ESP_OK;
            }
            else
            {
                data = strtok(NULL, "=&");
            }
            data = strtok(NULL, "=&");
        }
        free(contentLoc);
        return ESP_FAIL;
    }
    else
    {
        return ESP_FAIL;
    }
}

static const httpd_uri_t httpServerHandler_Get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = httpServerHandler_GetHandler,
};

esp_err_t httpServerHandler_PostHandler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    nvsHandler_err_t nvsWriteRet = NVS_WRITE_OK;
    size_t recv_size = req->content_len;
    char *content = malloc(req->content_len + 1);
    char *ssid = malloc(sizeof(char) * CONFIG_LEDCTRL_SSID_MAX_LENGTH);
    char *password = malloc(sizeof(char) * CONFIG_LEDCTRL_PASS_MAX_LENGTH);
    char *mqtt = malloc(sizeof(char) * CONFIG_LEDCTRL_MQTT_MAX_LENGTH);
    int retRecv = httpd_req_recv(req, content, recv_size);
    if (retRecv <= 0)
    { /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "content:%.*s", recv_size, content);
    ret = httpServerHandler_GetDataFromPost(content, "ssid", ssid);
    if (ESP_OK == ret)
    {
        ESP_LOGI(TAG, "Got SSID from HTTP Post:%s", ssid);
        nvsWriteRet = nvsHandler_saveWifiSSID(ssid);
    }
    ret = httpServerHandler_GetDataFromPost(content, "password", password);
    if (ESP_OK == ret)
    {
        ESP_LOGI(TAG, "Got Password from HTTP Post:%s", password);
        nvsWriteRet = nvsHandler_saveWifiPassword(password);
    }
    ret = httpServerHandler_GetDataFromPost(content, "mqtt", mqtt);
    if (ESP_OK == ret)
    {
        ESP_LOGI(TAG, "Got MQTT broker addr from HTTP Post:%s", mqtt);
        char mqttConverted[CONFIG_LEDCTRL_MQTT_MAX_LENGTH];
        httpServerHandler_UrlDecode(mqtt, mqttConverted);
        strcpy(mqtt, mqttConverted);
        nvsWriteRet = nvsHandler_saveMQTTaddr(mqtt);
    }
    /* Send a simple response */
    if (NVS_WRITE_OK == nvsWriteRet && ESP_OK == ret)
    {
        const char resp[] = "Config submitted and saved";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        mainStateHandler_setMainState(DEINIT_HTTP);
    }
    else
    {
        const char resp[] = "FAIL.";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    free(ssid);
    free(password);
    return ESP_OK;
}

static const httpd_uri_t httpServerHandler_Post = {
    .uri = "/post",
    .method = HTTP_POST,
    .handler = httpServerHandler_PostHandler,
    .user_ctx = NULL};

void httpServerHandler_StartServer(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (ESP_OK == httpd_start(&server, &config))
    {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &httpServerHandler_Get);
        httpd_register_uri_handler(server, &httpServerHandler_Post);
        mDNSHandler_StartMdnsService(config.server_port);
        httpServerHandle = server;
        ESP_LOGI(TAG, "Http server started");
    }
    else
    {
        ESP_LOGE(TAG, "Error starting server!");
    }
}

void httpServerHandler_StopServer(void)
{
    mDNSHandler_StopMdnsService();
    if (NULL != httpServerHandle)
    {
        httpd_stop(httpServerHandle);
    }
    else
    {
        ESP_LOGE(TAG, "error: httpServerHandle IS NULL");
    }
}
