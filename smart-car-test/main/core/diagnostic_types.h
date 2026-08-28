#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_events.h"
#include "motion_types.h"
#include "sensor_types.h"

typedef enum {
    DIAGNOSTIC_EVENT_MODE,
    DIAGNOSTIC_EVENT_OBSTACLE,
    DIAGNOSTIC_EVENT_STOP,
    DIAGNOSTIC_EVENT_SELF_TEST,
    DIAGNOSTIC_EVENT_INFO,
} diagnostic_event_kind_t;

typedef struct {
    diagnostic_event_kind_t kind;
    int64_t time_us;
    int32_t code;
    int32_t value;
    char text[64];
} diagnostic_event_t;

typedef struct {
    int64_t time_us;
    app_mode_t mode;
    uint8_t obstacle_state;
    uint8_t obstacle_clear_count;
    line_sensor_sample_t line;
    camera_line_snapshot_t camera;
    uint8_t line_pattern;
    int8_t line_state;
    int16_t line_error;
    int16_t line_control_error;
    int16_t line_base_speed;
    ultrasonic_snapshot_t ultrasonic;
    motor_command_t motor;
    int64_t encoder_a;
    int64_t encoder_b;
    int64_t encoder_c;
    bool button_raw;
    bool button_stable;
    bool button_armed;
    uint32_t control_overruns;
    uint32_t diagnostic_drops;
} diagnostic_snapshot_t;
