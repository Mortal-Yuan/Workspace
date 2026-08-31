#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config_types.h"
#include "sensor_types.h"

enum {
    CAMERA_LINE_VISION_MAX_HEIGHT = 60,
    CAMERA_LINE_VISION_MAX_ROI_PIXELS = 6400,
};

typedef struct {
    uint8_t mask[CAMERA_LINE_VISION_MAX_ROI_PIXELS];
    uint16_t queue[CAMERA_LINE_VISION_MAX_ROI_PIXELS];
    uint32_t histogram[256];
    uint32_t row_counts[CAMERA_LINE_VISION_MAX_HEIGHT];
    uint64_t row_sum_x[CAMERA_LINE_VISION_MAX_HEIGHT];
} camera_line_vision_workspace_t;

typedef struct {
    bool valid;
    bool line_detected;
    bool finish_detected;
    line_sensor_sample_t virtual_sensors;
    /* Near and far centers describe one accepted 8-connected component. */
    int16_t center_permille;
    int16_t far_center_permille;
    int16_t heading_permille;
    int16_t steering_permille;
    uint16_t width_permille;
    uint16_t component_height_permille;
    uint16_t component_area_permille;
    uint16_t black_permille;
    uint8_t connected_component_count;
    uint8_t threshold;
    uint8_t contrast;
} camera_line_analysis_t;

line_sensor_sample_t camera_line_virtual_sensors(int center_permille,
                                                 bool finish_detected);
int camera_line_steering_from_geometry(
    int near_center_permille, int far_center_permille,
    const camera_line_config_t *config);
camera_line_analysis_t camera_line_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace);
camera_line_analysis_t camera_line_analyze_rgb888_with_hint(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace,
    bool has_previous_line, int previous_center_permille,
    int previous_steering_permille);
camera_line_analysis_t camera_line_analyze_lower_half_rgb888(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace);
camera_line_analysis_t camera_line_analyze_lower_half_rgb888_with_hint(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace,
    bool has_previous_line, int previous_center_permille,
    int previous_steering_permille);
