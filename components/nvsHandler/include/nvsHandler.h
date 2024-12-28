#ifndef NVSHANDLER_H
#define NVSHANDLER_H
#include "esp_err.h"

typedef enum
{
    NVS_READ_OK,
    NVS_READ_FAIL,
    NVS_READ_VAL_UNINIT,
    NVS_WRITE_OK,
    NVS_WRITE_FAIL
} nvsHandler_err_t;

esp_err_t nvsHandler_InitNVS(void);

nvsHandler_err_t nvsHandler_saveWifiPassword(char *const pass);
nvsHandler_err_t nvsHandler_readWifiPassword(char *const pass);
nvsHandler_err_t nvsHandler_saveWifiSSID(char *const ssid);
nvsHandler_err_t nvsHandler_readWifiSSID(char *const ssid);
nvsHandler_err_t nvsHandler_saveMQTTaddr(char *const MQTTaddr);
nvsHandler_err_t nvsHandler_readMQTTaddr(char *const MQTTaddr);

#endif // NVSHANDLER_H