#include "motor_hal.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

bool motor_hal_preinit_safe(const motor_hal_config_t *config)
{
    const gpio_config_t enable = {
        .pin_bit_mask = 1ULL << config->enable_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&enable) == ESP_OK &&
        gpio_set_level(config->enable_pin, !config->enable_level) == ESP_OK;
}
bool motor_hal_init_outputs(const motor_hal_config_t *config)
{
    uint64_t mask = 0;
    for (int i = 0; i < 3; ++i) {
        mask |= 1ULL << config->in1_pin[i];
        mask |= 1ULL << config->in2_pin[i];
    }
    const gpio_config_t directions = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&directions) != ESP_OK) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (gpio_set_level(config->in1_pin[i], 0) != ESP_OK ||
            gpio_set_level(config->in2_pin[i], 0) != ESP_OK) {
            return false;
        }
    }

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = config->pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer) != ESP_OK) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = config->pwm_pin[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = config->pwm_channel[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        if (ledc_channel_config(&channel) != ESP_OK) {
            return false;
        }
    }
    return true;
}

bool motor_hal_set_enable(const motor_hal_config_t *config, bool enabled)
{
    return gpio_set_level(config->enable_pin,
                          enabled ? config->enable_level :
                                    !config->enable_level) == ESP_OK;
}

bool motor_hal_set_direction(const motor_hal_config_t *config,
                             int channel, int direction)
{
    return gpio_set_level(config->in1_pin[channel], direction > 0) == ESP_OK &&
        gpio_set_level(config->in2_pin[channel], direction < 0) == ESP_OK;
}

bool motor_hal_set_duty(const motor_hal_config_t *config,
                        int channel, uint32_t duty)
{
    const ledc_channel_t ledc_channel = config->pwm_channel[channel];
    return ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel, duty) == ESP_OK &&
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel) == ESP_OK;
}
