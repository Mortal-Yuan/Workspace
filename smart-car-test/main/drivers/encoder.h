#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct encoder_context encoder_context_t;

typedef struct {
    int a_pin;
    int b_pin;
    int64_t count;
    encoder_context_t *owner;
} encoder_channel_t;

struct encoder_context {
    encoder_channel_t channels[3];
    portMUX_TYPE lock;
    bool initialized;
};

typedef struct {
    int a_pin[3];
    int b_pin[3];
} encoder_config_t;

typedef struct {
    int64_t a;
    int64_t b;
    int64_t c;
    bool available;
} encoder_snapshot_t;

esp_err_t encoder_init(encoder_context_t *encoder,
                       const encoder_config_t *config);
encoder_snapshot_t encoder_snapshot(encoder_context_t *encoder);
void encoder_clear(encoder_context_t *encoder);
