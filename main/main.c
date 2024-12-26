#include "mainStateHandler.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "wifiHandler.h"
#include "mqttHandler.h"
#include "ledHandler.h"
#include "httpServerHandler.h"
#include <cJSON.h>
#include "nvsHandler.h"
#include "HomeAssistantHandler.h"

static char *TAG = "LED_Controller";

mainState_t mainState = INIT_STA;

void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_err_t ret = ESP_OK;
  ESP_ERROR_CHECK(esp_netif_init());
  nvsHandler_InitNVS();
  for (;;)
  {
    switch (mainState)
    {
    case INIT_STA:
      ret = ESP_OK;
      ret = wifi_init_sta();
      if (ESP_OK == ret)
      {
        mainState = INIT_CONTROLLER;
        ESP_LOGI(TAG, "WIFI_STA INIT SUCCESS");
      }
      else if (ESP_FAIL == ret)
      {
        mainState = DEINIT_STA;
        ESP_LOGI(TAG, "WIFI_STA INIT FAILED");
      }
      else if (ESP_ERR_NOT_ALLOWED == ret)
      {
        mainState = INIT_AP;
        ESP_LOGI(TAG, "WIFI_STA NO CREDENTIALS");
      }
      break;
    case INIT_CONTROLLER:
      ret = ESP_OK;
      HomeAssistantHandler_Init();

      ret = mqttHandler_Init();
      if (ESP_OK == ret)
      {
        ledHandler_Init();
        HomeAssistantHandler_InitLedStates();
        mainState = IDLE;
        ESP_LOGI(TAG, "INIT FINISHED");
      }
      else if (ESP_ERR_NOT_ALLOWED == ret)
      {
        mainState = DEINIT_STA;
        ESP_LOGI(TAG, "MQTT INIT NO BROKER ADDRES");
      }
      else if (ESP_OK != ret && ESP_ERR_NOT_ALLOWED != ret)
      {
        mainState = DEINIT_STA;
        ESP_LOGI(TAG, "MQTT INIT FAILED");
      }
      break;
    case DEINIT_STA:
      wifi_deinit_sta();
      ESP_LOGI(TAG, "WIFI_STA DEINIT SUCCESS");
      mainState = INIT_AP;
      break;
    case INIT_AP:
      wifi_init_ap();
      ESP_LOGI(TAG, "WIFI_AP INIT SUCCESS");
      mainState = INIT_HTTP;
      break;
    case DEINIT_AP:
      wifi_deinit_ap();
      ESP_LOGI(TAG, "WIFI_AP DEINIT SUCCESS");
      mainState = INIT_STA;
      break;
    case INIT_HTTP:
      (void)httpServerHandler_StartServer();
      ESP_LOGI(TAG, "HTTP INIT SUCCESS");
      mainState = IDLE;
      break;
    case DEINIT_HTTP:
      httpServerHandler_StopServer();
      ESP_LOGI(TAG, "HTTP DEINIT SUCCESS");
      mainState = DEINIT_AP;
      break;
    case IDLE:
      break;
    default:
      break;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
