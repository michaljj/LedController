#ifndef NVSHANDLER_H
#define NVSHANDLER_H
#include "esp_err.h"

typedef enum{
    NVS_READ_OK,
    NVS_READ_FAIL,
    NVS_READ_VAL_UNINIT,
    NVS_WRITE_OK,
    NVS_WRITE_FAIL
}nvsHandler_err_t;

esp_err_t nvsHandler_InitNVS(void);

nvsHandler_err_t nvsHandler_saveWifiPassword(const char* pass);
nvsHandler_err_t nvsHandler_readWifiPassword(const char* pass);
nvsHandler_err_t nvsHandler_saveWifiSSID(const char* ssid);
nvsHandler_err_t nvsHandler_readWifiSSID(const char* ssid);



















#endif //NVSHANDLER_H