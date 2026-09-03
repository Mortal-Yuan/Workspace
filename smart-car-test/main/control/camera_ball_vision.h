#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config_types.h"
#include "sensor_types.h"

enum {
    CAMERA_BALL_VISION_REFERENCE_WIDTH = 80,
    CAMERA_BALL_VISION_REFERENCE_HEIGHT = 60,
    CAMERA_BALL_VISION_MAX_WIDTH = 80,
    CAMERA_BALL_VISION_MAX_HEIGHT = 60,
    CAMERA_BALL_VISION_MAX_PIXELS =
        CAMERA_BALL_VISION_MAX_WIDTH * CAMERA_BALL_VISION_MAX_HEIGHT,
};

typedef struct {
    uint8_t mask[CAMERA_BALL_VISION_MAX_PIXELS];
    uint16_t queue[CAMERA_BALL_VISION_MAX_PIXELS];
} camera_ball_vision_workspace_t;

typedef struct {
    camera_ball_observation_t left_target;
    camera_ball_observation_t right_target;
} camera_blue_target_pair_t;

camera_ball_observation_t camera_ball_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace);

/* Analyze one color independently.  Calling this once for red and once for
 * blue lets the controller keep the ball and its destination in the same
 * decoded frame instead of selecting only the stronger of the two. */
camera_ball_observation_t camera_ball_analyze_color_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    ball_color_t requested_color, const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace);

/* Find the best blue component in each half of the image.  Keeping the two
 * selections independent prevents a large nearby target from hiding a
 * distant target that occupies only a few pixels. */
camera_blue_target_pair_t camera_blue_targets_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace);
