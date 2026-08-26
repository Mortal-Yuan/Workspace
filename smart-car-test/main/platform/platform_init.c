#include "platform_init.h"

#include "driver/gpio.h"

esp_err_t platform_install_gpio_isr_service(void)
{
    return gpio_install_isr_service(0);
}
