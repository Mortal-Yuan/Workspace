#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config_types.h"
#include "sensor_types.h"

typedef struct {
    bool valid;
    bool line_detected;
    bool finish_detected;
    line_sensor_sample_t virtual_sensors;
    int16_t center_permille;
    uint16_t width_permille;
    uint16_t black_permille;
    uint8_t threshold;
    uint8_t contrast;
} camera_line_analysis_t;

line_sensor_sample_t camera_line_virtual_sensors(int center_permille,
                                                 bool finish_detected);
camera_line_analysis_t camera_line_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config);
