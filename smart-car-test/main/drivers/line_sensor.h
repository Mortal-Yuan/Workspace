#pragma once

#include "esp_err.h"
#include "sensor_types.h"

typedef struct {
    int left_pin;
    int left_center_pin;
    int right_center_pin;
    int right_pin;
} line_sensor_config_t;

typedef struct {
    line_sensor_config_t config;
    bool initialized;
} line_sensor_t;

esp_err_t line_sensor_init(line_sensor_t *sensor,
                           const line_sensor_config_t *config);
line_sensor_sample_t line_sensor_read(const line_sensor_t *sensor);
