#ifndef MQTTHANDLER_H
#define MQTTHANDLER_H

#include "esp_system.h"
#include "esp_event.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "mqttHandler.h"
#include "HomeAssistantHandler.h"

#define CONFIG_BROKER_URL "mqtt://hass:Jurek744@192.168.100.24"



extern const char *TAG;



esp_err_t mqttPublish_Publish(const char *data, const char *topic, int qos, int retain);
//esp_err_t mqttHandler_GetLastMessage(const char *data, const char *topic);

void mqttHandler_AppStart(void);

#endif //MQTTHANDLER_H