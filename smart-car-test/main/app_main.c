#include "app_controller.h"
#include "app_config.h"

static app_controller_t s_controller;

void app_main(void)
{
    if (app_controller_init(&s_controller, &APP_CONFIG)) {
        app_controller_start(&s_controller);
    }
}
