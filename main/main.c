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
#include "ledHandler.h"
#include <cJSON.h>
static char* TAG = "LED_Controller";

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
    wifi_init_sta();
    mqttHandler_Init();
    ledHandler_Init();
    ESP_LOGI(TAG, "INIT FINISHED");

    for( ;; )
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

}
