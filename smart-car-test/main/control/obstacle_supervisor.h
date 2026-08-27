#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_types.h"
#include "motion_types.h"
#include "sensor_types.h"

typedef enum {
    OBSTACLE_STATE_SENSOR_CHECK,
    OBSTACLE_STATE_CLEAR,
    OBSTACLE_STATE_WAIT_CLEAR,
    OBSTACLE_STATE_BRAKE,
    OBSTACLE_STATE_STRAFE_LEFT_DISTANCE,
    OBSTACLE_STATE_SETTLE_FORWARD,
    OBSTACLE_STATE_FORWARD_DISTANCE,
    OBSTACLE_STATE_SETTLE_RIGHT,
    OBSTACLE_STATE_STRAFE_RIGHT_DISTANCE,
    OBSTACLE_STATE_LINE_CONFIRM,
    OBSTACLE_STATE_FINISHED,
    OBSTACLE_STATE_FAILSAFE,
} obstacle_state_t;

typedef enum {
    MOTION_POLICY_BLOCK,
    MOTION_POLICY_LINE_FOLLOW,
    MOTION_POLICY_OVERRIDE,
} motion_policy_t;

typedef enum {
    LINE_ACTION_KEEP,
    LINE_ACTION_SUSPEND,
    LINE_ACTION_RESUME,
} line_action_t;

typedef enum {
    OBSTACLE_TRANSITION_NONE,
    OBSTACLE_TRANSITION_TO_SENSOR_CHECK,
    OBSTACLE_TRANSITION_TO_CLEAR,
    OBSTACLE_TRANSITION_TO_WAIT_CLEAR,
    OBSTACLE_TRANSITION_TO_BRAKE,
    OBSTACLE_TRANSITION_TO_STRAFE_LEFT_DISTANCE,
    OBSTACLE_TRANSITION_TO_SETTLE_FORWARD,
    OBSTACLE_TRANSITION_TO_FORWARD_DISTANCE,
    OBSTACLE_TRANSITION_TO_SETTLE_RIGHT,
    OBSTACLE_TRANSITION_TO_STRAFE_RIGHT_DISTANCE,
    OBSTACLE_TRANSITION_TO_LINE_CONFIRM,
    OBSTACLE_TRANSITION_TO_FINISHED,
    OBSTACLE_TRANSITION_TO_FAILSAFE,
} obstacle_transition_t;

typedef enum {
    OBSTACLE_REASON_NONE,
    OBSTACLE_REASON_START,
    OBSTACLE_REASON_STARTUP_CLEAR,
    OBSTACLE_REASON_NEAR,
    OBSTACLE_REASON_UNCERTAIN,
    OBSTACLE_REASON_THREE_CLEAR,
    OBSTACLE_REASON_BRAKE_COMPLETE,
    OBSTACLE_REASON_EDGE_CLEAR,
    OBSTACLE_REASON_SEGMENT_COMPLETE,
    OBSTACLE_REASON_LINE_SEEN,
    OBSTACLE_REASON_LINE_LOST,
    OBSTACLE_REASON_LINE_CONFIRMED,
    OBSTACLE_REASON_FINISH_LINE,
    OBSTACLE_REASON_TIMEOUT,
    OBSTACLE_REASON_SENSOR_LOST,
} obstacle_reason_t;

typedef struct {
    motion_policy_t policy;
    body_motion_command_t override_motion;
    obstacle_transition_t transition;
    obstacle_reason_t reason;
    line_action_t line_action;
    uint8_t clear_count;
} obstacle_decision_t;

typedef struct {
    obstacle_config_t config;
    obstacle_state_t state;
    uint8_t clear_count;
    uint8_t uncertain_count;
    uint8_t no_echo_count;
    uint32_t last_seq;
    int64_t phase_started_us;
    bool bypass_completed;
} obstacle_supervisor_t;

void obstacle_supervisor_init(obstacle_supervisor_t *supervisor,
                              const obstacle_config_t *config);
void obstacle_supervisor_reset(obstacle_supervisor_t *supervisor);
obstacle_decision_t obstacle_supervisor_step(
    obstacle_supervisor_t *supervisor,
    const ultrasonic_event_t *event,
    line_sensor_sample_t line,
    int64_t now_us);
const char *obstacle_state_name(obstacle_state_t state);
