#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_types.h"
#include "motion_types.h"
#include "sensor_types.h"

typedef enum {
    BALL_APPROACH_STATE_IDLE,
    BALL_APPROACH_STATE_SEARCH_LEFT,
    BALL_APPROACH_STATE_SEARCH_RIGHT,
    BALL_APPROACH_STATE_SEARCH_SETTLE,
    BALL_APPROACH_STATE_ACQUIRE_STOP,
    BALL_APPROACH_STATE_GOAL_SEARCH_LEFT,
    BALL_APPROACH_STATE_GOAL_SEARCH_RIGHT,
    BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE,
    BALL_APPROACH_STATE_ROUTE_ALIGN,
    BALL_APPROACH_STATE_ROUTE_SHIFT,
    BALL_APPROACH_STATE_ROUTE_SETTLE,
    BALL_APPROACH_STATE_ALIGN,
    BALL_APPROACH_STATE_ALIGN_PULSE,
    BALL_APPROACH_STATE_ALIGN_SETTLE,
    BALL_APPROACH_STATE_APPROACH,
    BALL_APPROACH_STATE_CAPTURE_VERIFY,
    BALL_APPROACH_STATE_PUSH_ALIGN,
    BALL_APPROACH_STATE_PUSH_ALIGN_PULSE,
    BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE,
    BALL_APPROACH_STATE_PUSH,
    BALL_APPROACH_STATE_GOAL_VERIFY,
    BALL_APPROACH_STATE_DONE,
    BALL_APPROACH_STATE_FAILSAFE,
    BALL_APPROACH_STATE_RECOVERY_WAIT,
} ball_approach_state_t;

typedef enum {
    BALL_APPROACH_REASON_NONE,
    BALL_APPROACH_REASON_STARTED,
    BALL_APPROACH_REASON_SEARCH_REVERSE,
    BALL_APPROACH_REASON_CANDIDATE,
    BALL_APPROACH_REASON_CONFIRMED,
    BALL_APPROACH_REASON_GOAL_MISSING,
    BALL_APPROACH_REASON_ROUTE_SHIFT,
    BALL_APPROACH_REASON_ROUTE_ALIGNED,
    BALL_APPROACH_REASON_ROUTE_BEST_EFFORT,
    BALL_APPROACH_REASON_ALIGNMENT_PULSE,
    BALL_APPROACH_REASON_ALIGNED,
    BALL_APPROACH_REASON_REALIGN,
    BALL_APPROACH_REASON_LOST,
    BALL_APPROACH_REASON_CAPTURE_SEEN,
    BALL_APPROACH_REASON_CAPTURE_CONFIRMED,
    BALL_APPROACH_REASON_PUSH_STARTED,
    BALL_APPROACH_REASON_GOAL_SEEN,
    BALL_APPROACH_REASON_GOAL_REACHED,
    BALL_APPROACH_REASON_GOAL_OCCLUDED,
    BALL_APPROACH_REASON_CAMERA_STALE,
    BALL_APPROACH_REASON_ACQUIRE_TIMEOUT,
    BALL_APPROACH_REASON_PUSH_TIMEOUT,
    BALL_APPROACH_REASON_TOTAL_TIMEOUT,
} ball_approach_reason_t;

typedef enum {
    BALL_GOAL_PREFERENCE_LEFT,
    BALL_GOAL_PREFERENCE_RIGHT,
} ball_goal_preference_t;

typedef struct {
    motor_command_t command;
    ball_approach_state_t state;
    ball_approach_reason_t reason;
    int16_t center_error_permille;
    bool transitioned;
    bool done;
    bool failsafe;
} ball_approach_decision_t;

typedef struct {
    ball_approach_config_t config;
    kiwi_kinematics_config_t kinematics_config;
    ball_approach_state_t state;
    ball_approach_state_t settle_next_state;
    ball_approach_reason_t last_reason;
    motor_command_t held_command;
    camera_ball_observation_t selected_goal;
    uint32_t last_frame_seq;
    int16_t last_error_permille;
    uint8_t align_frames;
    uint8_t capture_frames;
    uint8_t capture_samples;
    uint8_t goal_frames;
    uint16_t push_align_motion_ms;
    uint16_t capture_reference_y_permille;
    uint16_t post_capture_min_y_permille;
    uint16_t search_motion_ms;
    int8_t turn_direction;
    bool push_align_turning;
    int64_t run_started_us;
    int64_t state_started_us;
    int64_t approach_started_us;
    int64_t route_started_us;
    int64_t push_started_us;
    int64_t push_motion_started_us;
    bool route_completed;
    bool goal_preference_locked;
    ball_color_t target_color;
    ball_goal_preference_t goal_preference;
    ball_approach_state_t initial_search_state;
    bool initialized;
} ball_approach_t;

void ball_approach_init(ball_approach_t *controller,
                        const ball_approach_config_t *config,
                        const kiwi_kinematics_config_t *kinematics_config);
void ball_approach_reset(ball_approach_t *controller);
void ball_approach_start(ball_approach_t *controller, int64_t now_us);
void ball_approach_start_for(ball_approach_t *controller,
                             ball_color_t target_color,
                             ball_goal_preference_t goal_preference,
                             int64_t now_us);
ball_approach_decision_t ball_approach_step(
    ball_approach_t *controller, const camera_line_snapshot_t *camera,
    int64_t now_us);
const char *ball_approach_state_name(ball_approach_state_t state);
