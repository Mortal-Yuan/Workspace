idf_component_register(
    SRCS "camera_uvc_stream_test.c"
    REQUIRES esp_driver_gpio esp_timer usb usb_host_uvc
)
