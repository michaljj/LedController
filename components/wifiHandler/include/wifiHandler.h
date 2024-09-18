#ifndef WIFIHANDLER_H
#define WIFIHANDLER_H
#include "esp_wifi.h"

#define ESP_WIFI_MAX_PASS_LENGHT CONFIG_LEDCTRL_PASS_MAX_LENGTH
#define ESP_WIFI_MAX_SSID_LENGHT CONFIG_LEDCTRL_SSID_MAX_LENGTH

#define ESP_MAXIMUM_RETRY       1 //DBG

#define ESP_WIFI_SSID_AP       "LEDControllerAP"
#define ESP_WIFI_PASS_AP       ""
#define ESP_WIFI_WIFI_CHANNEL  1
#define ESP_WIFI_MAX_STA_CONN  1


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1



esp_err_t wifi_init_sta(void);
void wifi_deinit_sta(void);

void wifi_init_ap(void);
void wifi_deinit_ap(void);


#endif //WIFIHANDLER_H