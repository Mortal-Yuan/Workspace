#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_types.h"
#include "esp_err.h"

typedef struct {
    int pin;
    start_button_config_t config;
    bool raw_pressed;
    bool stable_pressed;
    bool armed;
    bool pending_press;
    int64_t raw_changed_us;
    int64_t stable_changed_us;
    int64_t initialized_us;
    bool initialized;
} start_button_t;

esp_err_t start_button_init(start_button_t *button, int pin,
                            const start_button_config_t *config,
                            int64_t now_us);
bool start_button_update_level(start_button_t *button, bool pressed,
                               int64_t now_us);
bool start_button_poll(start_button_t *button, int64_t now_us);
