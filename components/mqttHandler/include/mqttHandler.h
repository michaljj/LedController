#ifndef MQTTHANDLER_H
#define MQTTHANDLER_H

#include "esp_system.h"
#include "esp_event.h"
#include "cJSON.h"
#include "esp_system.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "mqttHandler.h"

#define TOPIC_CFG "homeassistant/light/LightCtrl/config"
#define TOPIC_CMD "LedController/light/LightCtrl/set"
#define TOPIC_STAT "LedController/light/LightCtrl/state"
#define CONFIG_BROKER_URL "mqtt://hass:Jurek744@192.168.100.24"

typedef struct {
    char *platform;
    char *schema;
    char *name;
    char *unique_id;
    char *sw_ver;
    char *manufacturer;
    char *model;
    char *discovery_topic;
    char *state_topic;
    char *set_topic;
    bool brightness;
    bool color_temp;
    bool white_value;
    bool rgb;
    bool effect;
    bool retain;
    bool optimistic;
} esp_discovery_t;

typedef struct 
{
    bool power;
    int white;
    int temp;
    int brightness;
    int red;
    int green;
    int blue;
    int effect;
} esp_state_t;



extern const char *TAG;
extern esp_discovery_t esp_discovery;
extern esp_state_t esp_state;
extern cJSON *payload;
extern char *payload_cha;
extern bool discoverySend;
extern esp_mqtt_client_handle_t client;


char *esp_state_serialize(esp_state_t *state);
void mqtt_app_start(void);

#endif //MQTTHANDLER_H