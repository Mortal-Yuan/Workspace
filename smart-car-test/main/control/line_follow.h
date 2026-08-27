#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_types.h"
#include "motion_types.h"
#include "sensor_types.h"

typedef enum {
    LINE_STATE_IDLE,
    LINE_STATE_STRAIGHT,
    LINE_STATE_CURVE,
    LINE_STATE_SEARCH_LEFT,
    LINE_STATE_SEARCH_RIGHT,
} line_state_t;

typedef struct {
    line_follow_config_t config;
    line_state_t state;
    line_sensor_sample_t sensors;
    uint8_t pattern;
    uint8_t previous_pattern;
    unsigned pattern_stable_count;
    int active_count;
    int error;
    int control_error;
    int last_error;
    int locked_direction;
    int direction_candidate;
    unsigned direction_candidate_count;
    unsigned lost_count;
    int base_speed;
    int previous_target_a;
    int previous_target_c;
    int64_t assist_a_until_us;
    int64_t assist_c_until_us;
} line_follow_t;

void line_follow_init(line_follow_t *controller,
                      const line_follow_config_t *config);
void line_follow_reset_for_start(line_follow_t *controller,
                                 line_sensor_sample_t initial);
void line_follow_suspend(line_follow_t *controller);
void line_follow_resume(line_follow_t *controller);
motor_command_t line_follow_step(line_follow_t *controller,
                                 line_sensor_sample_t sensors,
                                 int speed_ceiling, int64_t now_us);
