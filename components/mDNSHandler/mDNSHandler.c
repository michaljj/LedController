#include <stdio.h>
#include "mDNSHandler.h"
#include "esp_system.h"
#include <esp_log.h>
#include "mdns.h"

static char* TAG = "MDNSHANDLER";


void mDNSHandler_StartMdnsService(uint16_t port)
{
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        ESP_LOGI(TAG, "MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    ESP_ERROR_CHECK(mdns_hostname_set(MDNS_HOSTNAME));
    ESP_LOGI(TAG, "mdns hostname set to: [%s]", MDNS_HOSTNAME);
    //set default instance
    ESP_ERROR_CHECK(mdns_instance_name_set(MDNS_INSTANCENAME));

    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", port, NULL, 0) );
}

void mDNSHandler_StopMdnsService(void)
{
    mdns_free();
}
