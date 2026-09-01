#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config_types.h"
#include "sensor_types.h"

enum {
    CAMERA_BALL_VISION_MAX_WIDTH = 80,
    CAMERA_BALL_VISION_MAX_HEIGHT = 60,
    CAMERA_BALL_VISION_MAX_PIXELS =
        CAMERA_BALL_VISION_MAX_WIDTH * CAMERA_BALL_VISION_MAX_HEIGHT,
};

typedef struct {
    uint8_t mask[CAMERA_BALL_VISION_MAX_PIXELS];
    uint16_t queue[CAMERA_BALL_VISION_MAX_PIXELS];
} camera_ball_vision_workspace_t;

camera_ball_observation_t camera_ball_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace);
