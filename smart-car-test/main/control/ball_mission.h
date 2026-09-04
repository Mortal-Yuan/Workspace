#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ball_approach.h"
#include "config_types.h"
#include "motion_types.h"
#include "sensor_types.h"

typedef enum {
    BALL_MISSION_STATE_IDLE,
    BALL_MISSION_STATE_RED_TO_LEFT,
    BALL_MISSION_STATE_TRANSITION_SETTLE,
    BALL_MISSION_STATE_GREEN_TO_RIGHT,
    BALL_MISSION_STATE_DONE,
    BALL_MISSION_STATE_FAILSAFE,
    /* Appended to preserve the diagnostic values of DONE and FAILSAFE. */
    BALL_MISSION_STATE_GREEN_ENTRY_TURN_RIGHT,
    BALL_MISSION_STATE_GREEN_ENTRY_TURN_SETTLE,
    BALL_MISSION_STATE_RED_EXIT_REVERSE,
    BALL_MISSION_STATE_RED_EXIT_REVERSE_SETTLE,
} ball_mission_state_t;

typedef struct {
    motor_command_t command;
    ball_approach_decision_t approach;
    ball_mission_state_t state;
    bool mission_transitioned;
    bool done;
    bool failsafe;
} ball_mission_decision_t;

typedef struct {
    ball_mission_config_t config;
    ball_mission_state_t state;
    int64_t state_started_us;
    bool initialized;
} ball_mission_t;

void ball_mission_init(ball_mission_t *mission,
                       const ball_mission_config_t *config);
void ball_mission_reset(ball_mission_t *mission);
void ball_mission_start(ball_mission_t *mission,
                        ball_approach_t *approach,
                        int64_t now_us);
ball_mission_decision_t ball_mission_step(
    ball_mission_t *mission, ball_approach_t *approach,
    const camera_line_snapshot_t *camera, int64_t now_us);
const char *ball_mission_state_name(ball_mission_state_t state);
