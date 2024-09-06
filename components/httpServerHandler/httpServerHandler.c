#include <stdio.h>
#include "httpServerHandler.h"

#include <string.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include "mDNSHandler.h"


static char* TAG = "HTTPSERVERHANDLER";

extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");

static esp_err_t hello_get_handler(httpd_req_t *req)
{
    const uint32_t root_len = root_end - root_start;

    ESP_LOGI(TAG, "Serve root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, root_start, root_len);

    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
};

httpd_handle_t httpServerHandler_StartServer()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        mDNSHandler_StartMdnsService(config.server_port);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}
