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
    LINE_STATE_REACQUIRE,
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
    int steering_permille;
    int last_steering_direction;
    int last_error;
    int locked_direction;
    int direction_candidate;
    unsigned direction_candidate_count;
    unsigned lost_count;
    int64_t lost_since_us;
    int search_direction;
    unsigned reacquire_count;
    uint32_t last_reacquire_frame_seq;
    uint32_t synthetic_frame_seq;
    int base_speed;
    int previous_target_a;
    int previous_target_c;
    int64_t assist_a_until_us;
    int64_t assist_c_until_us;
    motor_command_t last_cruise_command;
    motor_command_t lost_memory_command;
    bool last_cruise_valid;
    bool lost_memory_valid;
    int turn_memory_direction;
    int pending_turn_direction;
    int64_t pending_turn_since_us;
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
motor_command_t line_follow_step_camera(line_follow_t *controller,
                                        line_sensor_sample_t sensors,
                                        bool steering_valid,
                                        int steering_permille,
                                        uint32_t frame_seq,
                                        int speed_ceiling,
                                        int64_t now_us);
