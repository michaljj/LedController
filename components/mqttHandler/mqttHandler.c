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
#include "nvsHandler.h"

static char *TAG = "MQTTHANDLER";

static char *payload_cha = NULL;
static char MQTTaddr[ESP_WIFI_MAX_MQTTADDR_LENGHT];
static esp_mqtt_client_handle_t client;
static HomeAssistantHandler_Topics_t *HATopics;
static msgIds_t msgIds = {0, 0, 0, 0, 0};

esp_err_t mqttHandler_Publish(const char *data, const char *topic, int qos, int retain)
{
    if (data != NULL && topic != NULL)
    {
        esp_mqtt_client_publish(client, topic, data, 0, qos, retain);
        return ESP_OK;
    }
    else
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
            char *CfgTopic = HATopics->discoveryTopic;
            ESP_LOGI(TAG, "discovery topic: %s", CfgTopic);
            ret = mqttHandler_Publish(payload_cha, CfgTopic, 0, false);
            ESP_LOGI(TAG, "DISCOVERY_SENT");
            free(payload_cha);
        }
        else
        {
            ret = ESP_FAIL;
        }
    }
    else if (msgIds.CmdSubMsgId_aLed1 == MsgId || msgIds.CmdSubMsgId_dLed1 == MsgId || msgIds.CmdSubMsgId_dLed2 == MsgId)
    {
        ESP_LOGI(TAG, "CMD_SUBSCRIBED");
    }
    else if (msgIds.HaStatusSubMsgId == MsgId)
    {
        ESP_LOGI(TAG, "HA_STATUS_SUBSCRIBED");
    }
    else
    {
        ESP_LOGE(TAG, "IVALID_TOPIC");
    }
    return ret;
}

static esp_err_t mqttHandler_EventDataCbk(char *topic, int topicLen, char *data, int dataLen)
{
    if (0 == strncmp(topic, HATopics->setTopic_aLed1, topicLen))
    {
        ESP_LOGI(TAG, "TOPIC_CFG=%.*s", topicLen, topic);
        ESP_LOGI(TAG, "DATA=%.*s", dataLen, data);
        HomeAssistantHandler_HandleCmdMsg(data, dataLen, aLed1);
        char *statePayload = HomeAssistantHandler_GetState(aLed1);
        HomeAssistantHandler_SetState(statePayload, aLed1);
        mqttHandler_Publish(statePayload, HATopics->stateTopic_aLed1, 0, false);
        ESP_LOGI(TAG, "STATE=%s", statePayload);
    }
    if (0 == strncmp(topic, HATopics->setTopic_dLed1, topicLen))
    {
        ESP_LOGI(TAG, "TOPIC_CFG=%.*s", topicLen, topic);
        ESP_LOGI(TAG, "DATA=%.*s", dataLen, data);
        HomeAssistantHandler_HandleCmdMsg(data, dataLen, dLed1);
        char *statePayload = HomeAssistantHandler_GetState(dLed1);
        HomeAssistantHandler_SetState(statePayload, dLed1);
        mqttHandler_Publish(statePayload, HATopics->stateTopic_dLed1, 0, false);
        ESP_LOGI(TAG, "STATE=%s", statePayload);
    }
    if (0 == strncmp(topic, HATopics->setTopic_dLed2, topicLen))
    {
        ESP_LOGI(TAG, "TOPIC_CFG=%.*s", topicLen, topic);
        ESP_LOGI(TAG, "DATA=%.*s", dataLen, data);
        HomeAssistantHandler_HandleCmdMsg(data, dataLen, dLed2);
        char *statePayload = HomeAssistantHandler_GetState(dLed2);
        HomeAssistantHandler_SetState(statePayload, dLed2);
        mqttHandler_Publish(statePayload, HATopics->stateTopic_dLed2, 0, false);
        ESP_LOGI(TAG, "STATE=%s", statePayload);
    }

    else if (0 == strncmp(topic, HATopics->HAStatusTopic, topicLen))
    {
        ESP_LOGI(TAG, "TOPIC_CFG=%.*s", topicLen, topic);
        ESP_LOGI(TAG, "DATA=%.*s", dataLen, data);
    }
    else if ((0 == strncmp(topic, HATopics->discoveryTopic, topicLen)) || (0 == strncmp(topic, HATopics->stateTopic_aLed1, topicLen)))
    {
        //
    }
    else
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
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        msgIds.CfgSubMsgId = esp_mqtt_client_subscribe(client, HATopics->discoveryTopic, 0);
        msgIds.CmdSubMsgId_aLed1 = esp_mqtt_client_subscribe(client, HATopics->setTopic_aLed1, 0);
        msgIds.CmdSubMsgId_dLed1 = esp_mqtt_client_subscribe(client, HATopics->setTopic_dLed1, 0);
        msgIds.CmdSubMsgId_dLed2 = esp_mqtt_client_subscribe(client, HATopics->setTopic_dLed2, 0);
        msgIds.HaStatusSubMsgId = esp_mqtt_client_subscribe(client, HATopics->HAStatusTopic, 0);
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
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGE(TAG, "Last error reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Last error reported from tls stack: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG, "Last error captured as transport's socket errno: 0x%x", event->error_handle->esp_transport_sock_errno);
            ESP_LOGE(TAG, "Last errno string: %s", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_err_t mqttHandler_Init(void)
{
    nvsHandler_err_t nvsErr;
    nvsErr = nvsHandler_readMQTTaddr(MQTTaddr);
    if (NVS_READ_OK != nvsErr)
    {
        ESP_LOGE(TAG, "MQTT broker addr not in NVS");
        return ESP_ERR_NOT_ALLOWED;
    }
    esp_err_t ret = ESP_OK;
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "",
    };
    mqtt_cfg.broker.address.uri = malloc(sizeof(char) * (strlen(MQTTaddr) + 1));
    strcpy(mqtt_cfg.broker.address.uri, MQTTaddr);

    client = esp_mqtt_client_init(&mqtt_cfg);
    ret = esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqttHandler_EventHandler, NULL);
    ret = esp_mqtt_client_start(client);
    HATopics = HomeAssistantHandler_GetTopics();
    return ret;
}