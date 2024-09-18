#include <stdio.h>
#include <string.h>
#include "nvsHandler.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static char* TAG = "NVSHANDLER";

static nvs_handle_t nvsHandle;

esp_err_t nvsHandler_InitNVS(void)
{
    esp_err_t err = ESP_OK;
    err = nvs_open("storage", NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%d) opening NVS handle!", err);
    } else 
    {
        ESP_LOGI(TAG,"NVS open success");
    }

    return err;
}

nvsHandler_err_t nvsHandler_saveWifiPassword(const char *pass)
{
    if (0 != strlen(pass))
    {
        ESP_ERROR_CHECK(nvs_set_str(nvsHandle, "wifiPWD", pass));
        ESP_ERROR_CHECK(nvs_commit(nvsHandle));
        ESP_LOGI(TAG,"NVS password save success");
        return NVS_WRITE_OK;
    }else
    {
        ESP_LOGE(TAG,"NVS password save fail");
        return NVS_WRITE_FAIL;
    }
}

nvsHandler_err_t nvsHandler_readWifiPassword(const char *pass)
{
    if (NULL != pass)
    {
        esp_err_t err = ESP_OK;
        size_t passSize;
        err = nvs_get_str(nvsHandle, "wifiPWD", NULL, &passSize);
        char* pwd = malloc(passSize);
        err = nvs_get_str(nvsHandle, "wifiPWD", pwd, &passSize);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG,"NVS password read success");
            strcpy(pass, pwd);
            free(pwd);
            return NVS_READ_OK;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG,"NVS password read value not found");
            free(pwd);
            return NVS_READ_VAL_UNINIT;
            break;
        default:
            ESP_LOGE(TAG,"NVS password read fail");
            free(pwd);
            return NVS_READ_FAIL;
            break;
        }
    }else
    {
        ESP_LOGE(TAG,"NVS password read fail");
        return NVS_READ_FAIL;
    }
    
}

nvsHandler_err_t nvsHandler_saveWifiSSID(const char *ssid)
{
    if (0 != strlen(ssid))
    {
        ESP_ERROR_CHECK(nvs_set_str(nvsHandle, "wifiSSID", ssid));
        ESP_ERROR_CHECK(nvs_commit(nvsHandle));
        ESP_LOGI(TAG,"NVS SSID save success");
        return NVS_WRITE_OK;
    }else
    {
        ESP_LOGE(TAG,"NVS SSID save fail");
        return NVS_WRITE_FAIL;
    }
    
}

nvsHandler_err_t nvsHandler_readWifiSSID(const char *ssid)
{
    if (NULL != ssid)
    {
        esp_err_t err = ESP_OK;
        size_t ssidSize;
        err = nvs_get_str(nvsHandle, "wifiSSID", NULL, &ssidSize);
        char* ssidLoc = malloc(ssidSize);
        err = nvs_get_str(nvsHandle, "wifiSSID", ssidLoc, &ssidSize);

        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG,"NVS SSID read success");
            strcpy(ssid, ssidLoc);
            free(ssidLoc);
            return NVS_READ_OK;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG,"NVS SSID read value not found");
            free(ssidLoc);
            return NVS_READ_VAL_UNINIT;
            break;
        default:
            ESP_LOGE(TAG,"NVS SSID read fail");
            free(ssidLoc);
            return NVS_READ_FAIL;
            break;
        }
    }else
    {
        ESP_LOGE(TAG,"NVS SSID read fail, got null ptr");
        return NVS_READ_FAIL;
    }
    
}
