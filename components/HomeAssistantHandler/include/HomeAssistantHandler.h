#ifndef HAHANDLER_H
#define HAHANDLER_H

#include "ledHandler.h"

#define TOPIC_HA_STATUS "homeassistant/status"

typedef struct
{
    char *platform;
    char *schema;
    char *name;
    char *unique_id;
    char *sw_ver;
    char *hw_ver;
    char *manufacturer;
    char *model;
    char *discovery_topic;
    char *stateTopic_aLed1;
    char *setTopic_aLed1;
    char *stateTopic_dLed1;
    char *setTopic_dLed1;
    char *stateTopic_dLed2;
    char *setTopic_dLed2;
    bool brightness;
    bool color_temp;
    bool white_value;
    bool rgb;
    bool effect;
    bool retain;
    bool optimistic;
} HomeAssistantHandler_Discovery_t;

typedef struct
{
    char *state;
    int brightness;
    int red;
    int green;
    int blue;
} HomeAssistantHandler_State_t;

typedef struct
{
    char *HAStatusTopic;
    char *discoveryTopic;
    char *setTopic_aLed1;
    char *stateTopic_aLed1;
    char *setTopic_dLed1;
    char *stateTopic_dLed1;
    char *setTopic_dLed2;
    char *stateTopic_dLed2;
} HomeAssistantHandler_Topics_t;

typedef ledHandler_ledType_t HomeAssistantHandler_ledType_t;

void HomeAssistantHandler_Init();
void HomeAssistantHandler_InitLedStates();
HomeAssistantHandler_Topics_t *HomeAssistantHandler_GetTopics();
char *HomeAssistantHandler_GetDiscovery();
char *HomeAssistantHandler_GetState(HomeAssistantHandler_ledType_t ledType);
esp_err_t HomeAssistantHandler_SetState(char *statePayload, HomeAssistantHandler_ledType_t ledType);
esp_err_t HomeAssistantHandler_HandleCmdMsg(char *data, int dataLen, HomeAssistantHandler_ledType_t ledType);

#endif // HAHANDLER_H