#include "start_button.h"

#include <string.h>

#include "driver/gpio.h"

esp_err_t start_button_init(start_button_t *button, int pin,
                            const start_button_config_t *config,
                            int64_t now_us)
{
    if (button == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&input);
    if (result != ESP_OK) {
        return result;
    }
    const bool pressed = gpio_get_level(pin) == 0;
    *button = (start_button_t) {
        .pin = pin,
        .config = *config,
        .raw_pressed = pressed,
        .stable_pressed = pressed,
        .armed = !pressed,
        .raw_changed_us = now_us,
        .stable_changed_us = now_us,
        .initialized_us = now_us,
        .initialized = true,
    };
    return ESP_OK;
}

bool start_button_update_level(start_button_t *button, bool pressed,
                               int64_t now_us)
{
    if (button == NULL || !button->initialized) {
        return false;
    }
    if (pressed != button->raw_pressed) {
        button->raw_pressed = pressed;
        button->raw_changed_us = now_us;
    }
    if (button->raw_pressed != button->stable_pressed &&
        now_us - button->raw_changed_us >=
            button->config.press_debounce_ms * 1000LL) {
        button->stable_pressed = button->raw_pressed;
        button->stable_changed_us = now_us;
        if (button->stable_pressed && button->armed) {
            button->armed = false;
            if (now_us - button->initialized_us >=
                button->config.startup_guard_ms * 1000LL) {
                return true;
            }
            button->pending_press = true;
        }
    }
    if (!button->stable_pressed && !button->armed &&
        now_us - button->stable_changed_us >=
            button->config.release_rearm_ms * 1000LL) {
        button->armed = true;
    }
    if (button->pending_press &&
        now_us - button->initialized_us >=
            button->config.startup_guard_ms * 1000LL) {
        button->pending_press = false;
        return true;
    }
    return false;
}

bool start_button_poll(start_button_t *button, int64_t now_us)
{
    return start_button_update_level(button,
                                     gpio_get_level(button->pin) == 0,
                                     now_us);
}
