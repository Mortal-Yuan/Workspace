#include "encoder.h"

#include <string.h>

#include "driver/gpio.h"

static void IRAM_ATTR encoder_isr(void *arg)
{
    encoder_channel_t *channel = arg;
    const int a = gpio_get_level(channel->a_pin);
    const int b = gpio_get_level(channel->b_pin);
    portENTER_CRITICAL_ISR(&channel->owner->lock);
    channel->count += a == b ? 1 : -1;
    portEXIT_CRITICAL_ISR(&channel->owner->lock);
}
esp_err_t encoder_init(encoder_context_t *encoder,
                       const encoder_config_t *config)
{
    if (encoder == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(encoder, 0, sizeof(*encoder));
    encoder->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    uint64_t mask = 0;
    for (int i = 0; i < 3; ++i) {
        encoder->channels[i].a_pin = config->a_pin[i];
        encoder->channels[i].b_pin = config->b_pin[i];
        encoder->channels[i].owner = encoder;
        mask |= 1ULL << config->a_pin[i];
        mask |= 1ULL << config->b_pin[i];
    }
    const gpio_config_t input = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&input);
    if (result != ESP_OK) {
        return result;
    }
    for (int i = 0; i < 3; ++i) {
        result = gpio_set_intr_type(config->a_pin[i], GPIO_INTR_ANYEDGE);
        if (result == ESP_OK) {
            result = gpio_isr_handler_add(config->a_pin[i], encoder_isr,
                                          &encoder->channels[i]);
        }
        if (result != ESP_OK) {
            return result;
        }
    }
    encoder->initialized = true;
    return ESP_OK;
}

encoder_snapshot_t encoder_snapshot(encoder_context_t *encoder)
{
    encoder_snapshot_t result = {.available = encoder != NULL &&
                                               encoder->initialized};
    if (!result.available) {
        return result;
    }
    portENTER_CRITICAL(&encoder->lock);
    result.a = encoder->channels[0].count;
    result.b = encoder->channels[1].count;
    result.c = encoder->channels[2].count;
    portEXIT_CRITICAL(&encoder->lock);
    return result;
}

void encoder_clear(encoder_context_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return;
    }
    portENTER_CRITICAL(&encoder->lock);
    for (int i = 0; i < 3; ++i) {
        encoder->channels[i].count = 0;
    }
    portEXIT_CRITICAL(&encoder->lock);
}
