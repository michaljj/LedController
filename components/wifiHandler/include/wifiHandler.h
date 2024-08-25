#ifndef WIFIHANDLER_H
#define WIFIHANDLER_H


#define ESP_WIFI_SSID      "Michal_home_2_4"
#define ESP_WIFI_PASS      "Jurek744"
#define ESP_MAXIMUM_RETRY  10

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1



esp_err_t wifi_init_sta(void);
void wifi_deinit_sta(void);


#endif //WIFIHANDLER_H