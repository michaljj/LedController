#include <stdio.h>
#include "HomeAssistantHandler.h"

static esp_state_t HomeAssistantHandler_State = {
    .brightness = 100,
    .red = 255,
    .green = 255,
    .blue = 255,
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
    cJSON_AddStringToObject(root, "state", "ON");
    cJSON_AddNumberToObject(root, "brightness", state->brightness);
    cJSON *color = cJSON_AddObjectToObject(root, "color");
    cJSON_AddNumberToObject(color, "r", state->red);
    cJSON_AddNumberToObject(color, "g", state->green);
    cJSON_AddNumberToObject(color, "b", state->blue);
    if (state->white) {
        cJSON_AddNumberToObject(root, "white_value", state->white);
    }

    if (state->temp) {
        cJSON_AddNumberToObject(root, "color_temp", state->temp);
    }

    if (state->effect) {
        cJSON_AddNumberToObject(root, "effect", state->effect);
    }

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
            fprintf(stderr, "Error before: %s\n", error_ptr);
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
            fprintf(stderr, "Error before: %s\n", error_ptr);
        }
    }
    return state;
}

esp_err_t HomeAssistantHandler_SetState(esp_state_t* state_ptr, char* state)
{
    cJSON *stateJSON = cJSON_Parse(state);
    if (stateJSON == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            fprintf(stderr, "Error before: %s\n", error_ptr);
            return ESP_FAIL;
        }
    }
    free(state);
    cJSON *red = cJSON_GetObjectItemCaseSensitive(stateJSON, "red");
    cJSON *green = cJSON_GetObjectItemCaseSensitive(stateJSON, "green");
    cJSON *blue = cJSON_GetObjectItemCaseSensitive(stateJSON, "blue");
    cJSON *brightness = cJSON_GetObjectItemCaseSensitive(stateJSON, "brightness");
    cJSON_SetNumberValue(red, state_ptr->red);
    cJSON_SetNumberValue(green, state_ptr->green);
    cJSON_SetNumberValue(blue, state_ptr->blue);
    cJSON_SetNumberValue(brightness, state_ptr->brightness);
    state = cJSON_PrintUnformatted(stateJSON);

    if (state == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            fprintf(stderr, "Error before: %s\n", error_ptr);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}
