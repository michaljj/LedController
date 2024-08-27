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


static esp_state_t HomeAssistantHandler_State = {
    .state = "ON",
    .brightness = 100,
    .red = 255,
    .green = 255,
    .blue = 255
};

static esp_discovery_t HomeAssistantHandler_Discovery = {
        .name        = "LEDCtrl",
        .model       = "test",
        .sw_ver      = "0.0.1",
        .unique_id   = "LedController1",
        .set_topic   = TOPIC_CMD,
        .state_topic = TOPIC_STAT,
        .discovery_topic = TOPIC_CFG,
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


static char *HomeAssistantHandler_DiscoverySerialize(esp_discovery_t *discovery) 
{
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "platform", discovery->platform);
    cJSON_AddStringToObject(root, "schema", discovery->schema);
    cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
    cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
    char mac_char[19] = {0};
    HomeAssistantHandler_GetUniqeIdFromMac(mac_char);
    cJSON_AddStringToObject(root, "unique_id", mac_char);
    cJSON_AddBoolToObject(root, "brightness", discovery->brightness);
    cJSON_AddBoolToObject(root, "white_value", discovery->white_value);
    cJSON *supp_color_modes = cJSON_CreateArray();
    cJSON_AddItemToArray(supp_color_modes, cJSON_CreateString("rgb"));
    cJSON_AddItemToObject(root, "supported_color_modes", supp_color_modes);
    cJSON_AddBoolToObject(root, "effect"     , discovery->effect);
    cJSON_AddBoolToObject(root, "retain"     , discovery->retain);
    cJSON_AddBoolToObject(root, "optimistic" , discovery->optimistic);
    cJSON *dev = cJSON_AddObjectToObject(root, "device");
    cJSON_AddStringToObject(dev, "name", discovery->name);
    cJSON_AddStringToObject(dev, "model", discovery->model);
    cJSON_AddStringToObject(dev, "sw_version", discovery->sw_ver);
    cJSON_AddStringToObject(dev, "manufacturer", discovery->manufacturer);
    cJSON *identifiers = cJSON_AddArrayToObject(dev, "identifiers");
    cJSON_AddItemToArray(identifiers, cJSON_CreateString("01light"));

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json;
}

static char *HomeAssistantHandler_StateSerialize(esp_state_t *state) 
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