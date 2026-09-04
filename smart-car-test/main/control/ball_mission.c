#include "ball_mission.h"

#include <string.h>

static void enter_state(ball_mission_t *mission,
                        ball_mission_decision_t *decision,
                        ball_mission_state_t state, int64_t now_us)
{
    if (mission->state == state) return;
    mission->state = state;
    mission->state_started_us = now_us;
    decision->mission_transitioned = true;
}

void ball_mission_init(ball_mission_t *mission,
                       const ball_mission_config_t *config)
{
    if (mission == NULL || config == NULL) return;
    memset(mission, 0, sizeof(*mission));
    mission->config = *config;
    mission->state = BALL_MISSION_STATE_IDLE;
    mission->initialized = true;
}

void ball_mission_reset(ball_mission_t *mission)
{
    if (mission == NULL) return;
    const ball_mission_config_t config = mission->config;
    const bool initialized = mission->initialized;
    memset(mission, 0, sizeof(*mission));
    mission->config = config;
    mission->state = BALL_MISSION_STATE_IDLE;
    mission->initialized = initialized;
}

void ball_mission_start(ball_mission_t *mission,
                        ball_approach_t *approach,
                        int64_t now_us)
{
    if (mission == NULL || approach == NULL || !mission->initialized) return;
    ball_mission_reset(mission);
    mission->state = BALL_MISSION_STATE_RED_TO_LEFT;
    mission->state_started_us = now_us;
    ball_approach_start_for(approach, BALL_COLOR_RED,
                            BALL_GOAL_PREFERENCE_LEFT, now_us);
}

ball_mission_decision_t ball_mission_step(
    ball_mission_t *mission, ball_approach_t *approach,
    const camera_line_snapshot_t *camera, int64_t now_us)
{
    ball_mission_decision_t decision = {
        .command = motor_command_zero(),
        .state = mission != NULL ? mission->state :
                                   BALL_MISSION_STATE_FAILSAFE,
    };
    if (mission == NULL || approach == NULL || camera == NULL ||
        !mission->initialized || !approach->initialized) {
        decision.failsafe = true;
        return decision;
    }

    switch (mission->state) {
    case BALL_MISSION_STATE_RED_TO_LEFT:
        decision.approach = ball_approach_step(approach, camera, now_us);
        decision.command = decision.approach.command;
        if (decision.approach.failsafe) {
            ball_approach_reset(approach);
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_FAILSAFE, now_us);
        } else if (decision.approach.done) {
            ball_approach_reset(approach);
            decision.command = motor_command_zero();
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_TRANSITION_SETTLE, now_us);
        }
        break;

    case BALL_MISSION_STATE_TRANSITION_SETTLE:
        if (now_us - mission->state_started_us >=
            mission->config.transition_settle_ms * 1000LL) {
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_RED_EXIT_REVERSE, now_us);
            decision.command = (motor_command_t) {
                (int16_t)mission->config.red_exit_reverse_boost_speed,
                0,
                (int16_t)mission->config.red_exit_reverse_boost_speed,
            };
        }
        break;

    case BALL_MISSION_STATE_RED_EXIT_REVERSE: {
        const int64_t elapsed_us = now_us - mission->state_started_us;
        if (elapsed_us >= mission->config.red_exit_reverse_ms * 1000LL) {
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_RED_EXIT_REVERSE_SETTLE, now_us);
        } else {
            const int speed = elapsed_us <
                    mission->config.red_exit_reverse_boost_ms * 1000LL ?
                mission->config.red_exit_reverse_boost_speed :
                mission->config.red_exit_reverse_speed;
            decision.command = (motor_command_t) {
                (int16_t)speed, 0, (int16_t)speed,
            };
        }
        break;
    }

    case BALL_MISSION_STATE_RED_EXIT_REVERSE_SETTLE:
        if (now_us - mission->state_started_us >=
            mission->config.red_exit_reverse_settle_ms * 1000LL) {
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_GREEN_ENTRY_TURN_RIGHT, now_us);
            decision.command = (motor_command_t) {
                (int16_t)mission->config.green_entry_right_turn_speed,
                (int16_t)-mission->config.green_entry_right_turn_speed,
                (int16_t)-mission->config.green_entry_right_turn_speed,
            };
        }
        break;

    case BALL_MISSION_STATE_GREEN_ENTRY_TURN_RIGHT:
        if (now_us - mission->state_started_us >=
            mission->config.green_entry_right_turn_ms * 1000LL) {
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_GREEN_ENTRY_TURN_SETTLE, now_us);
        } else {
            decision.command = (motor_command_t) {
                (int16_t)mission->config.green_entry_right_turn_speed,
                (int16_t)-mission->config.green_entry_right_turn_speed,
                (int16_t)-mission->config.green_entry_right_turn_speed,
            };
        }
        break;

    case BALL_MISSION_STATE_GREEN_ENTRY_TURN_SETTLE:
        if (now_us - mission->state_started_us >=
            mission->config.green_entry_turn_settle_ms * 1000LL) {
            ball_approach_start_for(approach, BALL_COLOR_GREEN,
                                    BALL_GOAL_PREFERENCE_RIGHT, now_us);
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_GREEN_TO_RIGHT, now_us);
        }
        break;

    case BALL_MISSION_STATE_GREEN_TO_RIGHT:
        decision.approach = ball_approach_step(approach, camera, now_us);
        decision.command = decision.approach.command;
        if (decision.approach.failsafe) {
            ball_approach_reset(approach);
            enter_state(mission, &decision,
                        BALL_MISSION_STATE_FAILSAFE, now_us);
        } else if (decision.approach.done) {
            ball_approach_reset(approach);
            decision.command = motor_command_zero();
            enter_state(mission, &decision, BALL_MISSION_STATE_DONE, now_us);
        }
        break;

    case BALL_MISSION_STATE_DONE:
    case BALL_MISSION_STATE_FAILSAFE:
    case BALL_MISSION_STATE_IDLE:
        break;
    }

    decision.state = mission->state;
    decision.done = mission->state == BALL_MISSION_STATE_DONE;
    decision.failsafe = mission->state == BALL_MISSION_STATE_FAILSAFE;
    if (mission->state == BALL_MISSION_STATE_TRANSITION_SETTLE ||
        mission->state == BALL_MISSION_STATE_RED_EXIT_REVERSE_SETTLE ||
        mission->state == BALL_MISSION_STATE_GREEN_ENTRY_TURN_SETTLE ||
        mission->state == BALL_MISSION_STATE_DONE ||
        mission->state == BALL_MISSION_STATE_FAILSAFE ||
        mission->state == BALL_MISSION_STATE_IDLE) {
        decision.command = motor_command_zero();
    }
    return decision;
}

const char *ball_mission_state_name(ball_mission_state_t state)
{
    switch (state) {
    case BALL_MISSION_STATE_IDLE: return "MISSION_IDLE";
    case BALL_MISSION_STATE_RED_TO_LEFT: return "MISSION_RED_TO_LEFT";
    case BALL_MISSION_STATE_TRANSITION_SETTLE:
        return "MISSION_TRANSITION_SETTLE";
    case BALL_MISSION_STATE_GREEN_TO_RIGHT:
        return "MISSION_GREEN_TO_RIGHT";
    case BALL_MISSION_STATE_DONE: return "MISSION_DONE";
    case BALL_MISSION_STATE_FAILSAFE: return "MISSION_FAILSAFE";
    case BALL_MISSION_STATE_GREEN_ENTRY_TURN_RIGHT:
        return "MISSION_GREEN_ENTRY_TURN_RIGHT_60";
    case BALL_MISSION_STATE_GREEN_ENTRY_TURN_SETTLE:
        return "MISSION_GREEN_ENTRY_TURN_SETTLE";
    case BALL_MISSION_STATE_RED_EXIT_REVERSE:
        return "MISSION_RED_EXIT_REVERSE_16CM";
    case BALL_MISSION_STATE_RED_EXIT_REVERSE_SETTLE:
        return "MISSION_RED_EXIT_REVERSE_SETTLE";
    default: return "MISSION_UNKNOWN";
    }
}
