#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "camera_line_vision.h"
#include "config_types.h"

enum {
    CAMERA_PREVIEW_WIDTH = 80,
    CAMERA_PREVIEW_HEIGHT = 60,
    CAMERA_PREVIEW_PIXEL_COUNT =
        CAMERA_PREVIEW_WIDTH * CAMERA_PREVIEW_HEIGHT,
    CAMERA_PREVIEW_VERSION = 1,
    CAMERA_PREVIEW_PIXEL_FORMAT_RGB332 = 1,
    CAMERA_PREVIEW_FLAG_FRAME_VALID = 1U << 0,
    CAMERA_PREVIEW_FLAG_LINE_DETECTED = 1U << 1,
    CAMERA_PREVIEW_FLAG_FINISH_DETECTED = 1U << 2,
    CAMERA_PREVIEW_FLAG_BALL_CANDIDATE = 1U << 3,
    CAMERA_PREVIEW_FLAG_BALL_DETECTED = 1U << 4,
};

/* This structure is the USB-UART wire format. ESP32-S3 and the PC monitor
 * are little-endian. Keep this packed and update the Python header
 * format whenever fields change. */
typedef struct __attribute__((packed)) {
    uint8_t magic[8];
    uint8_t version;
    uint8_t width;
    uint8_t height;
    uint8_t pixel_format;
    uint8_t flags;
    uint8_t threshold;
    uint8_t contrast;
    uint8_t reserved;
    uint32_t sequence;
    uint32_t timestamp_ms;
    int16_t center_permille;
    int16_t far_center_permille;
    int16_t steering_permille;
    uint16_t payload_size;
    uint32_t payload_crc32;
    uint8_t pixels[CAMERA_PREVIEW_PIXEL_COUNT];
} camera_preview_packet_t;

typedef void (*camera_preview_sink_t)(
    void *context, const camera_preview_packet_t *packet);

uint32_t camera_preview_crc32(const uint8_t *data, size_t length);
bool camera_preview_build_rgb332(
    camera_preview_packet_t *packet,
    const uint8_t *rgb888, size_t width, size_t height,
    const camera_line_config_t *config,
    const camera_line_analysis_t *analysis,
    const camera_ball_observation_t *ball,
    const camera_ball_observation_t *left_target,
    const camera_ball_observation_t *right_target,
    uint32_t sequence, uint32_t timestamp_ms);
