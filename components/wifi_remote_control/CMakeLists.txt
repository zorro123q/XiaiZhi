idf_component_register(
    SRCS "wifi_remote_control.cc"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_wifi esp_netif esp_timer lwip
)
