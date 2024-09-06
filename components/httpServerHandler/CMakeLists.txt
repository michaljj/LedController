idf_component_register(SRCS "httpServerHandler.c"
                    INCLUDE_DIRS "include" REQUIRES esp_http_server mDNSHandler EMBED_FILES root.html)
