#include "line_sensor.h"

#include <string.h>

#include "driver/gpio.h"

esp_err_t line_sensor_init(line_sensor_t *sensor,
                           const line_sensor_config_t *config)
{
    if (sensor == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;
    const gpio_config_t input = {
        .pin_bit_mask = (1ULL << config->left_pin) |
                        (1ULL << config->left_center_pin) |
                        (1ULL << config->right_center_pin) |
                        (1ULL << config->right_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const esp_err_t result = gpio_config(&input);
    sensor->initialized = result == ESP_OK;
    return result;
}
line_sensor_sample_t line_sensor_read(const line_sensor_t *sensor)
{
    if (sensor == NULL || !sensor->initialized) {
        return (line_sensor_sample_t) {0};
    }
    return (line_sensor_sample_t) {
        .left = gpio_get_level(sensor->config.left_pin) == 0,
        .left_center = gpio_get_level(sensor->config.left_center_pin) == 0,
        .right_center = gpio_get_level(sensor->config.right_center_pin) == 0,
        .right = gpio_get_level(sensor->config.right_pin) == 0,
    };
}
