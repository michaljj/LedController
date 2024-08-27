#ifndef HAHANDLER_H
#define HAHANDLER_H



#define TOPIC_CFG "homeassistant/light/LightCtrl/config"
#define TOPIC_CMD "LedController/light/LightCtrl/set"
#define TOPIC_STAT "LedController/light/LightCtrl/state"
#define TOPIC_HA_STATUS "homeassistant/status"


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
    char* state;
    int brightness;
    int red;
    int green;
    int blue;
} esp_state_t;

 
char *HomeAssistantHandler_GetDiscovery();
char *HomeAssistantHandler_GetState();
esp_err_t HomeAssistantHandler_SetState(char* statePayload);
esp_err_t HomeAssistantHandler_HandleCmdMsg(char* data, int dataLen);



#endif //HAHANDLER_H