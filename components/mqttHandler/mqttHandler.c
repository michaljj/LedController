#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "mqttHandler.h"

bool discoverySend = false;
char* payload_cha = NULL;
esp_mqtt_client_handle_t client;

esp_discovery_t esp_discovery = {
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

esp_state_t esp_state =
{
    .brightness = 100,
    .red = 255,
    .green = 255,
    .blue = 255,
};

cJSON *payload = NULL;


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}


char *esp_discovery_serialize(esp_discovery_t *discovery) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "platform", discovery->platform);
    cJSON_AddStringToObject(root, "schema", discovery->schema);
    cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
    cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
    cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
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
    cJSON_AddItemToArray(identifiers, cJSON_CreateString("1231235434"));

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json;
}

char *esp_state_serialize(esp_state_t *state) {
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


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int)event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        msg_id = esp_mqtt_client_subscribe(client, TOPIC_CFG, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, TOPIC_CMD, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        if (discoverySend == false)
        {
            payload_cha = esp_discovery_serialize(&esp_discovery);
            if (payload_cha != NULL)
            {
                printf("%s\n", payload_cha);
                msg_id = esp_mqtt_client_publish(client, TOPIC_CFG, payload_cha, 0, 0, false);
                discoverySend = true;
                free(payload_cha);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            }
            else
            {
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL)
                {
                    fprintf(stderr, "Error before: %s\n", error_ptr);
                }
            }

        }
        
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC_CFG=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}