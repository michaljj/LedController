#include <stdio.h>
#include <string.h>
#include "nvsHandler.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static char *TAG = "NVSHANDLER";

static nvs_handle_t nvsHandle;

esp_err_t nvsHandler_InitNVS(void)
{
    esp_err_t ret = ESP_OK;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ESP_OK == ret)
    {
        ret = nvs_open("storage", NVS_READWRITE, &nvsHandle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%d) opening NVS handle!", ret);
        }
        else
        {
            ESP_LOGI(TAG, "NVS open success");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error (%d) opening NVS handle!", ret);
    }
    return ret;
}

nvsHandler_err_t nvsHandler_saveWifiPassword(char *const pass)
{
    if (0 != strlen(pass))
    {
        ESP_ERROR_CHECK(nvs_set_str(nvsHandle, "wifiPWD", pass));
        ESP_ERROR_CHECK(nvs_commit(nvsHandle));
        ESP_LOGI(TAG, "NVS password save success");
        return NVS_WRITE_OK;
    }
    else
    {
        ESP_LOGE(TAG, "NVS password save fail");
        return NVS_WRITE_FAIL;
    }
}

nvsHandler_err_t nvsHandler_readWifiPassword(char *const pass)
{
    if (NULL != pass)
    {
        esp_err_t ret = ESP_OK;
        size_t passSize;
        ret = nvs_get_str(nvsHandle, "wifiPWD", NULL, &passSize);
        char *pwd = malloc(passSize);
        ret = nvs_get_str(nvsHandle, "wifiPWD", pwd, &passSize);
        switch (ret)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "NVS password read success");
            strcpy(pass, pwd);
            free(pwd);
            return NVS_READ_OK;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "NVS password read value not found");
            free(pwd);
            return NVS_READ_VAL_UNINIT;
            break;
        default:
            ESP_LOGE(TAG, "NVS password read fail");
            free(pwd);
            return NVS_READ_FAIL;
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "NVS password read fail");
        return NVS_READ_FAIL;
    }
}

nvsHandler_err_t nvsHandler_saveWifiSSID(char *const ssid)
{
    if (0 != strlen(ssid))
    {
        ESP_ERROR_CHECK(nvs_set_str(nvsHandle, "wifiSSID", ssid));
        ESP_ERROR_CHECK(nvs_commit(nvsHandle));
        ESP_LOGI(TAG, "NVS SSID save success");
        return NVS_WRITE_OK;
    }
    else
    {
        ESP_LOGE(TAG, "NVS SSID save fail");
        return NVS_WRITE_FAIL;
    }
}

nvsHandler_err_t nvsHandler_readWifiSSID(char *const ssid)
{
    if (NULL != ssid)
    {
        esp_err_t ret = ESP_OK;
        size_t ssidSize;
        ret = nvs_get_str(nvsHandle, "wifiSSID", NULL, &ssidSize);
        char *ssidLoc = malloc(ssidSize);
        ret = nvs_get_str(nvsHandle, "wifiSSID", ssidLoc, &ssidSize);

        switch (ret)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "NVS SSID read success");
            strcpy(ssid, ssidLoc);
            free(ssidLoc);
            return NVS_READ_OK;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "NVS SSID read value not found");
            free(ssidLoc);
            return NVS_READ_VAL_UNINIT;
            break;
        default:
            ESP_LOGE(TAG, "NVS SSID read fail");
            free(ssidLoc);
            return NVS_READ_FAIL;
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "NVS SSID read fail, got null ptr");
        return NVS_READ_FAIL;
    }
}

nvsHandler_err_t nvsHandler_saveMQTTaddr(char *const MQTTaddr)
{
    if (0 != strlen(MQTTaddr))
    {
        ESP_ERROR_CHECK(nvs_set_str(nvsHandle, "MQTTaddr", MQTTaddr));
        ESP_ERROR_CHECK(nvs_commit(nvsHandle));
        ESP_LOGI(TAG, "NVS MQTTaddr save success");
        return NVS_WRITE_OK;
    }
    else
    {
        ESP_LOGE(TAG, "NVS MQTTaddr save fail");
        return NVS_WRITE_FAIL;
    }
}

nvsHandler_err_t nvsHandler_readMQTTaddr(char *const MQTTaddr)
{
    if (NULL != MQTTaddr)
    {
        esp_err_t ret = ESP_OK;
        size_t MQTTaddrSize;
        ret = nvs_get_str(nvsHandle, "MQTTaddr", NULL, &MQTTaddrSize);
        char *MQTTaddrLoc = malloc(MQTTaddrSize);
        ret = nvs_get_str(nvsHandle, "MQTTaddr", MQTTaddrLoc, &MQTTaddrSize);
        ESP_LOGI(TAG, "NVS MQTTaddr len: %d value: %s", MQTTaddrSize, MQTTaddrLoc);

        switch (ret)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "NVS MQTTaddr read success");
            strcpy(MQTTaddr, MQTTaddrLoc);
            free(MQTTaddrLoc);
            return NVS_READ_OK;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "NVS MQTTaddr read value not found");
            free(MQTTaddrLoc);
            return NVS_READ_VAL_UNINIT;
            break;
        default:
            ESP_LOGE(TAG, "NVS MQTTaddr read fail");
            free(MQTTaddrLoc);
            return NVS_READ_FAIL;
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "NVS MQTTaddr read fail, got null ptr");
        return NVS_READ_FAIL;
    }
}
