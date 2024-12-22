#include <stdio.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "string.h"
#include "math.h"
#include "cJSON.h"
#include "ledHandler.h"

#include "HomeAssistantHandler.h"

static char* TAG = "HOMEASSISTANTHANDLER";

static HomeAssistantHandler_Topics_t* HomeAssistantHandler_Topics_ptr = NULL;

static HomeAssistantHandler_State_t HomeAssistantHandler_State = {
    .state = "ON",
    .brightness = 100,
    .red = 255,
    .green = 255,
    .blue = 255
};

static HomeAssistantHandler_Discovery_t HomeAssistantHandler_Discovery = {
        .name        = "LedController",
        .manufacturer = "Jurek electronics",
        .model       = "MidiV1",
        .sw_ver      = "0.1",
        .hw_ver      = "0.1",
        .unique_id   = "LedController1",
        .schema      = "json",
        .platform    = "mqtt",
        .brightness  = true,
        .color_temp  = true,
        .white_value = false,
        .rgb         = true,
        .effect      = false,
        .retain      = false,
        .optimistic  = false,
};


static void HomeAssistantHandler_GetUniqeIdFromMac(char* mac_char)
{
    uint8_t mac_addr[6] = {0};
    esp_efuse_mac_get_default(mac_addr);
    sprintf(mac_char, "%02X%02X%02X%02X%02X%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

static char *HomeAssistantHandler_GetDiscoveryTopic()
{
    static char *discoveryTopic = NULL;
    if (NULL == discoveryTopic)
    {
        char str[80];
        char mac_char[19] = {0};
        HomeAssistantHandler_GetUniqeIdFromMac(mac_char);
        strcpy(str, "homeassistant/device/");
        strcat(str, mac_char);
        strcat(str, "/LightCtrl/config");
        size_t strLen = strlen(str);
        discoveryTopic = malloc(sizeof(char) * (strLen + 1));
        strcpy(discoveryTopic, str);
    }
    return discoveryTopic;
}

static char *HomeAssistantHandler_GetStateTopic(char* entityId)
{
    static char *StateTopic = NULL;
    if (NULL == StateTopic || NULL == strstr(StateTopic, entityId))
    {
        char str[80];
        char mac_char[19] = {0};
        HomeAssistantHandler_GetUniqeIdFromMac(mac_char);
        strcpy(str, "LedController/");
        strcat(str, mac_char);
        strcat(str, "/");
        strcat(str, entityId);
        strcat(str, "/config");
        size_t strLen = strlen(str);
        StateTopic = malloc(sizeof(char) * (strLen + 1));
        strcpy(StateTopic, str);
    }
    ESP_LOGI(TAG, "StateTopic: %s", StateTopic);
    return StateTopic;
}

static char *HomeAssistantHandler_GetSetTopic(char* entityId)
{
    static char *SetTopic = NULL;
    if (NULL == SetTopic || NULL == strstr(SetTopic, entityId))
    {
        char str[80];
        char mac_char[19] = {0};
        HomeAssistantHandler_GetUniqeIdFromMac(mac_char);
        strcpy(str, "LedController/");
        strcat(str, mac_char);
        strcat(str, "/");
        strcat(str, entityId);
        strcat(str, "/set");
        size_t strLen = strlen(str);
        SetTopic = malloc(sizeof(char) * (strLen + 1));
        strcpy(SetTopic, str);
    }
    ESP_LOGI(TAG, "SetTopic: %s", SetTopic);
    return SetTopic;
}

static char *HomeAssistantHandler_DiscoverySerialize(HomeAssistantHandler_Discovery_t *discovery) 
{
    char mac_char[19] = {0};
    HomeAssistantHandler_GetUniqeIdFromMac(mac_char);
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    //dev
    cJSON *dev = cJSON_AddObjectToObject(root, "dev");
    cJSON *identifiers = cJSON_AddArrayToObject(dev, "ids");
    cJSON_AddItemToArray(identifiers, cJSON_CreateString(mac_char));
    cJSON_AddStringToObject(dev, "name", discovery->name);
    cJSON_AddStringToObject(dev, "mf", discovery->manufacturer);
    cJSON_AddStringToObject(dev, "model", discovery->model);
    cJSON_AddStringToObject(dev, "sw", discovery->sw_ver);
    cJSON_AddStringToObject(dev, "sn", mac_char);
    cJSON_AddStringToObject(dev, "hw", discovery->hw_ver);
    //orig
    cJSON *orig = cJSON_AddObjectToObject(root, "o");
    cJSON_AddStringToObject(orig, "name", discovery->name);
    //cmps
    cJSON *cmps = cJSON_AddObjectToObject(root, "cmps");
    //aLed1 -> cmps
    cJSON *aLed1 = cJSON_AddObjectToObject(cmps, "aLed1");
    cJSON_AddStringToObject(aLed1, "p", "light");
    cJSON_AddStringToObject(aLed1, "unique_id", "aLed1");
    cJSON_AddStringToObject(aLed1, "schema", discovery->schema);
    cJSON_AddBoolToObject(aLed1, "brightness", discovery->brightness);
    cJSON *supp_color_modes = cJSON_CreateArray();
    cJSON_AddItemToArray(supp_color_modes, cJSON_CreateString("rgb"));
    cJSON_AddItemToObject(aLed1, "supported_color_modes", supp_color_modes);
    cJSON_AddStringToObject(aLed1, "state_topic", discovery->stateTopic_aLed1);
    cJSON_AddStringToObject(aLed1, "command_topic", discovery->setTopic_aLed1);

    json = cJSON_PrintUnformatted(root);
    ESP_LOGI(TAG, "disc payload: %s", json); //DEBUG
    cJSON_Delete(root);

    return json;
}

static char *HomeAssistantHandler_StateSerialize(HomeAssistantHandler_State_t *state) 
{
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", state->state);
    cJSON_AddNumberToObject(root, "brightness", state->brightness);
    cJSON *color = cJSON_AddObjectToObject(root, "color");
    cJSON_AddNumberToObject(color, "r", state->red);
    cJSON_AddNumberToObject(color, "g", state->green);
    cJSON_AddNumberToObject(color, "b", state->blue);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json;
}

void HomeAssistantHandler_Init()
{
    HomeAssistantHandler_Topics_ptr = malloc(sizeof(HomeAssistantHandler_Topics_t));
    HomeAssistantHandler_Topics_ptr->discoveryTopic = strdup(HomeAssistantHandler_GetDiscoveryTopic());
    HomeAssistantHandler_Topics_ptr->HAStatusTopic = strdup(TOPIC_HA_STATUS);
    HomeAssistantHandler_Topics_ptr->setTopic_aLed1 = strdup(HomeAssistantHandler_GetSetTopic("aLed1"));
    HomeAssistantHandler_Topics_ptr->stateTopic_aLed1 = strdup(HomeAssistantHandler_GetStateTopic("aLed1"));
    HomeAssistantHandler_Discovery.setTopic_aLed1 = HomeAssistantHandler_Topics_ptr->setTopic_aLed1;
    HomeAssistantHandler_Discovery.stateTopic_aLed1 = HomeAssistantHandler_Topics_ptr->stateTopic_aLed1;
    HomeAssistantHandler_Discovery.discovery_topic = HomeAssistantHandler_Topics_ptr->discoveryTopic;

}

HomeAssistantHandler_Topics_t *HomeAssistantHandler_GetTopics()
{
    return HomeAssistantHandler_Topics_ptr;
}

char *HomeAssistantHandler_GetDiscovery()
{
    char *discovery = NULL;
    discovery = HomeAssistantHandler_DiscoverySerialize(&HomeAssistantHandler_Discovery);
    if (discovery == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "Error before: %s\n", error_ptr);
        }
    }
    return discovery;
}


char *HomeAssistantHandler_GetState()
{
    char *state = NULL;
    state = HomeAssistantHandler_StateSerialize(&HomeAssistantHandler_State);
    if (state == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "Error before: %s\n", error_ptr);
        }
    }
    return state;
}

esp_err_t HomeAssistantHandler_SetState(char* statePayload)
{
    cJSON *stateJSON = cJSON_Parse(statePayload);
    if (stateJSON == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "Error before: %s\n", error_ptr);
            return ESP_FAIL;
        }
    }
    free(statePayload);
    cJSON *state = cJSON_GetObjectItemCaseSensitive(stateJSON, "state");
    cJSON *red = cJSON_GetObjectItemCaseSensitive(stateJSON, "red");
    cJSON *green = cJSON_GetObjectItemCaseSensitive(stateJSON, "green");
    cJSON *blue = cJSON_GetObjectItemCaseSensitive(stateJSON, "blue");
    cJSON *brightness = cJSON_GetObjectItemCaseSensitive(stateJSON, "brightness");
    cJSON_SetValuestring(state, HomeAssistantHandler_State.state);
    cJSON_SetNumberValue(red, HomeAssistantHandler_State.red);
    cJSON_SetNumberValue(green, HomeAssistantHandler_State.green);
    cJSON_SetNumberValue(blue, HomeAssistantHandler_State.blue);
    cJSON_SetNumberValue(brightness, HomeAssistantHandler_State.brightness);
    statePayload = cJSON_PrintUnformatted(stateJSON);

    if (statePayload == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "Error before: %s\n", error_ptr);
            return ESP_FAIL;
        }
    }
    cJSON_free(state);
    cJSON_free(red);
    cJSON_free(green);
    cJSON_free(blue);
    cJSON_free(brightness);
    return ESP_OK;
}

esp_err_t HomeAssistantHandler_HandleCmdMsg(char* data, int dataLen)
{
    cJSON *dataJSON = cJSON_ParseWithLength(data, dataLen);
    cJSON *state = cJSON_GetObjectItemCaseSensitive(dataJSON, "state");
    cJSON *brightness = cJSON_GetObjectItemCaseSensitive(dataJSON, "brightness");
    cJSON *color = cJSON_GetObjectItemCaseSensitive(dataJSON, "color");



    char *stateCh = cJSON_GetStringValue(state);
    if (stateCh != NULL)
    {
        if (0 == strncmp(stateCh, "ON", 2))
        {
            ledHandler_SetOnOff(true);
        }else if (0 == strncmp(stateCh, "OFF", 3))
        {
            ledHandler_SetOnOff(false);
        }
        
        
        HomeAssistantHandler_State.state = stateCh;
    }
    double brightnessInt = cJSON_GetNumberValue(brightness);
    if (brightnessInt != NAN && brightnessInt <= 255)
    {
        HomeAssistantHandler_State.brightness = (int)brightnessInt;
        ledHandler_SetRGB(HomeAssistantHandler_State.red, HomeAssistantHandler_State.green, HomeAssistantHandler_State.blue, HomeAssistantHandler_State.brightness);
    }
    if (color != NULL)
    {
        cJSON *red = cJSON_GetObjectItemCaseSensitive(color, "r");
        cJSON *green = cJSON_GetObjectItemCaseSensitive(color, "g");
        cJSON *blue = cJSON_GetObjectItemCaseSensitive(color, "b");
        double redInt = cJSON_GetNumberValue(red);
        if (redInt != NAN && redInt <= 255)
        {
            HomeAssistantHandler_State.red = (int)redInt;
        }
        double greenInt = cJSON_GetNumberValue(green);
        if (greenInt != NAN && greenInt <= 255)
        {
            HomeAssistantHandler_State.green = (int)greenInt;
        }
        double blueInt = cJSON_GetNumberValue(blue);
        if (blueInt != NAN && blueInt <= 255)
        {
            HomeAssistantHandler_State.blue = (int)blueInt;
        }
        ledHandler_SetRGB(HomeAssistantHandler_State.red, HomeAssistantHandler_State.green, HomeAssistantHandler_State.blue, HomeAssistantHandler_State.brightness);
    }
    

    cJSON_free(dataJSON);
    cJSON_free(state);
    cJSON_free(brightness);
    cJSON_free(color);
    return ESP_OK;

}