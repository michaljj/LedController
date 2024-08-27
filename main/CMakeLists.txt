idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "" REQUIRES sht4x esp_event nvs_flash json wifiHandler mqttHandler ledHandler)
