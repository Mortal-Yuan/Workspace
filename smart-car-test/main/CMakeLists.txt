idf_component_register(
    SRCS
        "app_main.c"
        "app/app_config.c"
        "app/app_controller.c"
        "control/kiwi_kinematics.c"
        "control/line_follow.c"
        "control/obstacle_supervisor.c"
        "diagnostics/diagnostics.c"
        "drivers/encoder.c"
        "drivers/line_sensor.c"
        "drivers/motor_driver.c"
        "drivers/start_button.c"
        "drivers/ultrasonic.c"
        "platform/motor_hal_espidf.c"
        "platform/platform_init.c"
    INCLUDE_DIRS
        "."
        "app"
        "control"
        "core"
        "diagnostics"
        "drivers"
        "platform"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart
             esp_driver_usb_serial_jtag esp_timer
)
