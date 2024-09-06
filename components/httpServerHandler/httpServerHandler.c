#include <stdio.h>
#include "httpServerHandler.h"

#include <string.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include "mDNSHandler.h"


static char* TAG = "HTTPSERVERHANDLER";

extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");

static esp_err_t httpServerHandler_GetHandler(httpd_req_t *req)
{
    const uint32_t root_len = root_end - root_start;

    ESP_LOGI(TAG, "Serve root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, root_start, root_len);

    return ESP_OK;
}

static const httpd_uri_t httpServerHandler_Get = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = httpServerHandler_GetHandler,
};

esp_err_t httpServerHandler_PostHandler(httpd_req_t *req)
{

    size_t recv_size = req->content_len;
    char* content = malloc(req->content_len + 1);

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
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
    /* Send a simple response */
    const char resp[] = "Config submitted";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t httpServerHandler_Post = {
    .uri      = "/post",
    .method   = HTTP_POST,
    .handler  = httpServerHandler_PostHandler,
    .user_ctx = NULL
};

httpd_handle_t httpServerHandler_StartServer()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &httpServerHandler_Get);
        httpd_register_uri_handler(server, &httpServerHandler_Post);
        mDNSHandler_StartMdnsService(config.server_port);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}
