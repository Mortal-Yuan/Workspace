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
