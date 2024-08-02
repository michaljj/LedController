#ifndef HAHANDLER_H
#define HAHANDLER_H

#include "esp_system.h"
#include "cJSON.h"
#include "esp_mac.h"



#define TOPIC_CFG "homeassistant/light/LightCtrl/config"
#define TOPIC_CMD "LedController/light/LightCtrl/set"
#define TOPIC_STAT "LedController/light/LightCtrl/state"



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

 
char *HomeAssistantHandler_GetDiscovery();
char *HomeAssistantHandler_GetState();
esp_err_t HomeAssistantHandler_SetState(esp_state_t* state_ptr, char* state);




#endif //HAHANDLER_H