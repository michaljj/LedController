#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "wifiHandler.h"
#include "mqttHandler.h"
#include "mqtt_client.h"

#include <cJSON.h>
#define TOPIC "homeassistant/light/12345/config"
const char *TAG = "LED_Controller";
char *payload_cha2;

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    wifi_init_sta();
    mqtt_app_start();

    int msg_id;
  
    for( ;; )
    {
      if (discoverySend == true)
      {
        if (payload_cha2 == NULL)
        {
          payload_cha2 = esp_state_serialize(&esp_state);
        }
            printf("%s\n", payload_cha2);
            msg_id = esp_mqtt_client_publish(client, TOPIC_STAT, payload_cha2, 0, 0, false);
            ESP_LOGI(TAG, "sent state successful, msg_id=%d", msg_id);
      }
      
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

}
