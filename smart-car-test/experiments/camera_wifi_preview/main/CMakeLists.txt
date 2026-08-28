idf_component_register(
    SRCS "camera_wifi_preview.c"
    REQUIRES esp_driver_gpio esp_http_server esp_netif esp_wifi nvs_flash
             usb usb_host_uvc
)
