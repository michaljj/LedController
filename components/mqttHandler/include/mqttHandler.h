#ifndef MQTTHANDLER_H
#define MQTTHANDLER_H



#define CONFIG_BROKER_URL "mqtt://hass:Jurek744@192.168.100.24"
#define ESP_WIFI_MAX_MQTTADDR_LENGHT CONFIG_LEDCTRL_MQTT_MAX_LENGTH


typedef struct
{
    int CfgSubMsgId;
    int CmdSubMsgId;
    int HaStatusSubMsgId;
}msgIds_t;



esp_err_t mqttPublish_Publish(const char *data, const char *topic, int qos, int retain);

esp_err_t mqttHandler_Init(void);

#endif //MQTTHANDLER_H