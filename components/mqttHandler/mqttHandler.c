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
#include "HomeAssistantHandler.h"
#include "mqttHandler.h"

static char* TAG = "MQTTHANDLER";

static char* payload_cha = NULL;
static esp_mqtt_client_handle_t client;
static msgIds_t msgIds = {0,0,0};

esp_err_t mqttHandler_Publish(const char *data, const char *topic, int qos, int retain)
{
    if (data != NULL && topic != NULL)
    {
        esp_mqtt_client_publish(client, topic, data, 0, qos, retain);
        return ESP_OK;
    }else
    {
        return ESP_FAIL;
    }
}

static esp_err_t mqttHandler_EventSubsctibedCbk(int MsgId)
{
    esp_err_t ret = ESP_OK;
    if (msgIds.CfgSubMsgId == MsgId)
    {
        payload_cha = HomeAssistantHandler_GetDiscovery();
        if (payload_cha != NULL)
        {
            ret = mqttHandler_Publish(payload_cha, TOPIC_CFG, 0, false);
            ESP_LOGI(TAG, "DISCOVERY_SENT");
            free(payload_cha);

        }else
        {
            ret = ESP_FAIL;
        }
    }else if (msgIds.CmdSubMsgId == MsgId)
    {
        ESP_LOGI(TAG, "CMD_SUBSCRIBED");
    }else if (msgIds.HaStatusSubMsgId == MsgId)
    {
        ESP_LOGI(TAG, "HA_STATUS_SUBSCRIBED");
    }else
    {
        ESP_LOGE(TAG, "IVALID_TOPIC");
    }
    return ret;
    
}

static esp_err_t mqttHandler_EventDataCbk(char* topic, int topicLen, char* data, int dataLen)
{
    if (0 == strncmp(topic, TOPIC_CMD, topicLen))
    {
        ESP_LOGI(TAG, "TOPIC_CFG=%.*s", topicLen, topic);
        ESP_LOGI(TAG, "DATA=%.*s", dataLen, data);
        HomeAssistantHandler_HandleCmdMsg(data, dataLen);
        char* statePayload = HomeAssistantHandler_GetState();
        HomeAssistantHandler_SetState(statePayload);
        mqttHandler_Publish(statePayload, TOPIC_STAT, 0, false);
        ESP_LOGI(TAG, "STATE=%s", statePayload);

    }else if (0 == strncmp(topic, TOPIC_HA_STATUS, topicLen))
    {
        ESP_LOGI(TAG, "TOPIC_CFG=%.*s", topicLen, topic);
        ESP_LOGI(TAG, "DATA=%.*s", dataLen, data);
    }else if ((0 == strncmp(topic, TOPIC_CFG, topicLen)) || (0 == strncmp(topic, TOPIC_STAT, topicLen)))
    {
        //
    }else 
    {
        return ESP_FAIL;
    }
    
    return ESP_OK;
    
}

static void mqttHandler_EventHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int)event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        msgIds.CfgSubMsgId = esp_mqtt_client_subscribe(client, TOPIC_CFG, 0);
        msgIds.CmdSubMsgId = esp_mqtt_client_subscribe(client, TOPIC_CMD, 0);
        msgIds.HaStatusSubMsgId = esp_mqtt_client_subscribe(client, TOPIC_HA_STATUS, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        ESP_ERROR_CHECK(mqttHandler_EventSubsctibedCbk(event->msg_id));
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        mqttHandler_EventDataCbk(event->topic, event->topic_len, event->data, event->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Last error reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Last error reported from tls stack: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG, "Last error captured as transport's socket errno: 0x%x",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGE(TAG, "Last errno string: %s", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


void mqttHandler_Init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqttHandler_EventHandler, NULL);
    esp_mqtt_client_start(client);
}