#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool left;
    bool left_center;
    bool right_center;
    bool right;
} line_sensor_sample_t;

static inline uint8_t line_sensor_pattern(line_sensor_sample_t sample)
{
    return (uint8_t)((sample.left << 3) |
                     (sample.left_center << 2) |
                     (sample.right_center << 1) |
                     sample.right);
}

typedef struct {
    line_sensor_sample_t virtual_sensors;
    bool streaming;
    bool frame_valid;
    bool fresh;
    bool line_detected;
    bool finish_detected;
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
    uint32_t received_frames;
    uint32_t decoded_frames;
    uint32_t dropped_frames;
    uint32_t decode_errors;
    int64_t updated_us;
} camera_line_snapshot_t;

typedef enum {
    ULTRASONIC_QUALITY_NEVER,
    ULTRASONIC_QUALITY_VALID,
    ULTRASONIC_QUALITY_OUTLIER,
    ULTRASONIC_QUALITY_LOST,
    ULTRASONIC_QUALITY_INVALID,
    /* A complete Echo pulse was received, but it represents >4 m/no return. */
    ULTRASONIC_QUALITY_NO_RETURN,
} ultrasonic_quality_t;

typedef struct {
    uint32_t seq;
    int64_t completed_us;
    bool has_echo;
    int32_t pulse_us;
    int32_t raw_mm;
    int32_t filtered_mm;
    ultrasonic_quality_t quality;
    bool echo_high;
    bool safety_uncertain;
} ultrasonic_event_t;

typedef struct {
    uint32_t seq;
    int64_t updated_us;
    int32_t raw_mm;
    int32_t filtered_mm;
    ultrasonic_quality_t quality;
    bool echo_high;
    bool waiting;
    uint32_t timeout_count;
    uint32_t anomaly_count;
} ultrasonic_snapshot_t;
