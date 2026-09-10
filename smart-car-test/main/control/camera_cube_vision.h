#pragma once
#include "cube_grab.h"
#define CUBE_PIXELS (160*120)
typedef struct {
    uint8_t color[CUBE_PIXELS], dark[CUBE_PIXELS], mask[CUBE_PIXELS];
    uint8_t work[CUBE_PIXELS], seen[CUBE_PIXELS];
    uint16_t queue[CUBE_PIXELS];
} camera_cube_workspace_t;
cube_observation_t camera_cube_detect(camera_cube_workspace_t *w,
    const uint8_t *rgb332, uint32_t sequence, int64_t timestamp_us);

cube_observation_t camera_yellow_detect(camera_cube_workspace_t *w,
    const uint8_t *rgb332, uint32_t sequence, int64_t timestamp_us);
