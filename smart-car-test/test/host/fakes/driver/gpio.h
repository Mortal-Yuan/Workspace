#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint64_t pin_bit_mask;
    int mode;
    int pull_up_en;
    int pull_down_en;
    int intr_type;
} gpio_config_t;

enum {
    GPIO_MODE_INPUT = 1,
    GPIO_MODE_OUTPUT = 2,
    GPIO_PULLUP_DISABLE = 0,
    GPIO_PULLUP_ENABLE = 1,
    GPIO_PULLDOWN_DISABLE = 0,
    GPIO_PULLDOWN_ENABLE = 1,
    GPIO_INTR_DISABLE = 0,
    GPIO_INTR_ANYEDGE = 3,
};

esp_err_t gpio_config(const gpio_config_t *config);
int gpio_get_level(int pin);
esp_err_t gpio_set_level(int pin, int level);
esp_err_t gpio_isr_handler_add(int pin, void (*handler)(void *), void *arg);
