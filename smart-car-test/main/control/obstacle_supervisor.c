#include "obstacle_supervisor.h"

#include <string.h>

typedef enum {
    OBSERVATION_NONE,
    OBSERVATION_NEAR,
    OBSERVATION_CLEAR,
    OBSERVATION_UNCERTAIN,
} observation_t;

static bool line_detected(line_sensor_sample_t line)
{
    return line.left || line.left_center || line.right_center || line.right;
}

static obstacle_transition_t transition_for(obstacle_state_t state)
{
    switch (state) {
    case OBSTACLE_STATE_SENSOR_CHECK:
        return OBSTACLE_TRANSITION_TO_SENSOR_CHECK;
    case OBSTACLE_STATE_CLEAR:
        return OBSTACLE_TRANSITION_TO_CLEAR;
    case OBSTACLE_STATE_WAIT_CLEAR:
        return OBSTACLE_TRANSITION_TO_WAIT_CLEAR;
    case OBSTACLE_STATE_BRAKE:
        return OBSTACLE_TRANSITION_TO_BRAKE;
    case OBSTACLE_STATE_STRAFE_LEFT_EDGE:
        return OBSTACLE_TRANSITION_TO_STRAFE_LEFT_EDGE;
    case OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE:
        return OBSTACLE_TRANSITION_TO_STRAFE_LEFT_CLEARANCE;
    case OBSTACLE_STATE_SETTLE_FORWARD:
        return OBSTACLE_TRANSITION_TO_SETTLE_FORWARD;
    case OBSTACLE_STATE_FORWARD_BYPASS:
        return OBSTACLE_TRANSITION_TO_FORWARD_BYPASS;
    case OBSTACLE_STATE_SETTLE_RIGHT:
        return OBSTACLE_TRANSITION_TO_SETTLE_RIGHT;
    case OBSTACLE_STATE_STRAFE_RIGHT_LINE:
        return OBSTACLE_TRANSITION_TO_STRAFE_RIGHT_LINE;
    case OBSTACLE_STATE_LINE_CONFIRM:
        return OBSTACLE_TRANSITION_TO_LINE_CONFIRM;
    case OBSTACLE_STATE_FAILSAFE:
        return OBSTACLE_TRANSITION_TO_FAILSAFE;
    default:
        return OBSTACLE_TRANSITION_NONE;
    }
}

static void enter_state(obstacle_supervisor_t *supervisor,
                        obstacle_decision_t *decision,
                        obstacle_state_t state,
                        obstacle_reason_t reason,
                        int64_t now_us)
{
    supervisor->state = state;
    supervisor->phase_started_us = now_us;
    supervisor->clear_count = 0;
    supervisor->uncertain_count = 0;
    decision->transition = transition_for(state);
    decision->reason = reason;
}

static observation_t classify_event(obstacle_supervisor_t *supervisor,
                                    const ultrasonic_event_t *event)
{
    if (event == NULL || event->seq == 0 ||
        event->seq == supervisor->last_seq) {
        return OBSERVATION_NONE;
    }
    supervisor->last_seq = event->seq;
    if (event->has_echo && event->raw_mm >= 20 &&
        event->raw_mm <= supervisor->config.stop_mm) {
        return OBSERVATION_NEAR;
    }
    const bool far = event->has_echo &&
                     event->quality == ULTRASONIC_QUALITY_VALID &&
                     event->raw_mm > supervisor->config.stop_mm &&
                     event->filtered_mm > supervisor->config.stop_mm &&
                     !event->safety_uncertain;
    const bool no_echo_clear = !event->has_echo && !event->echo_high &&
                               !event->safety_uncertain;
    if (far || no_echo_clear) return OBSERVATION_CLEAR;
    if (event->safety_uncertain || event->echo_high ||
        event->quality == ULTRASONIC_QUALITY_INVALID ||
        event->quality == ULTRASONIC_QUALITY_OUTLIER) {
        return OBSERVATION_UNCERTAIN;
    }
    return OBSERVATION_NONE;
}

static void count_clear(obstacle_supervisor_t *supervisor,
                        observation_t observation, int limit)
{
    if (observation == OBSERVATION_CLEAR) {
        if (supervisor->clear_count < (uint8_t)limit) {
            supervisor->clear_count++;
        }
    } else if (observation != OBSERVATION_NONE) {
        supervisor->clear_count = 0;
    }
}

static bool maneuver_state(obstacle_state_t state)
{
    return state == OBSTACLE_STATE_STRAFE_LEFT_EDGE ||
           state == OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE ||
           state == OBSTACLE_STATE_FORWARD_BYPASS ||
           state == OBSTACLE_STATE_STRAFE_RIGHT_LINE;
}

static void apply_policy(const obstacle_supervisor_t *supervisor,
                         obstacle_decision_t *decision, int64_t now_us)
{
    decision->policy = MOTION_POLICY_BLOCK;
    decision->override_motion = body_motion_zero();
    int speed;
    switch (supervisor->state) {
    case OBSTACLE_STATE_CLEAR:
        decision->policy = MOTION_POLICY_LINE_FOLLOW;
        break;
    case OBSTACLE_STATE_STRAFE_LEFT_EDGE:
        speed = now_us - supervisor->phase_started_us <
                supervisor->config.motion_boost_ms * 1000LL ?
                supervisor->config.lateral_start_speed :
                supervisor->config.lateral_speed;
        decision->policy = MOTION_POLICY_OVERRIDE;
        decision->override_motion.left = (int16_t)speed;
        break;
    case OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE:
        decision->policy = MOTION_POLICY_OVERRIDE;
        decision->override_motion.left =
            (int16_t)supervisor->config.lateral_speed;
        break;
    case OBSTACLE_STATE_FORWARD_BYPASS:
        speed = now_us - supervisor->phase_started_us <
                supervisor->config.motion_boost_ms * 1000LL ?
                supervisor->config.forward_start_speed :
                supervisor->config.forward_speed;
        decision->policy = MOTION_POLICY_OVERRIDE;
        decision->override_motion.forward = (int16_t)speed;
        break;
    case OBSTACLE_STATE_STRAFE_RIGHT_LINE:
        speed = now_us - supervisor->line_search_started_us <
                supervisor->config.motion_boost_ms * 1000LL ?
                supervisor->config.lateral_start_speed :
                supervisor->config.lateral_speed;
        decision->policy = MOTION_POLICY_OVERRIDE;
        decision->override_motion.left = (int16_t)-speed;
        break;
    default:
        break;
    }
    decision->clear_count = supervisor->clear_count;
}

void obstacle_supervisor_init(obstacle_supervisor_t *supervisor,
                              const obstacle_config_t *config)
{
    memset(supervisor, 0, sizeof(*supervisor));
    supervisor->config = *config;
    supervisor->state = OBSTACLE_STATE_SENSOR_CHECK;
}

void obstacle_supervisor_reset(obstacle_supervisor_t *supervisor)
{
    supervisor->state = OBSTACLE_STATE_SENSOR_CHECK;
    supervisor->clear_count = 0;
    supervisor->uncertain_count = 0;
    supervisor->last_seq = 0;
    supervisor->phase_started_us = 0;
    supervisor->line_search_started_us = 0;
}

obstacle_decision_t obstacle_supervisor_step(
    obstacle_supervisor_t *supervisor,
    const ultrasonic_event_t *event,
    line_sensor_sample_t line,
    int64_t now_us)
{
    obstacle_decision_t decision = {0};
    const observation_t observation = classify_event(supervisor, event);
    const int64_t elapsed_us = now_us - supervisor->phase_started_us;

    if (maneuver_state(supervisor->state)) {
        if (observation == OBSERVATION_UNCERTAIN) {
            if (supervisor->uncertain_count <
                (uint8_t)supervisor->config.uncertain_limit) {
                supervisor->uncertain_count++;
            }
        } else if (observation != OBSERVATION_NONE) {
            supervisor->uncertain_count = 0;
        }
        if (supervisor->uncertain_count >=
            (uint8_t)supervisor->config.uncertain_limit) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_FAILSAFE,
                        OBSTACLE_REASON_UNCERTAIN, now_us);
            decision.line_action = LINE_ACTION_SUSPEND;
            apply_policy(supervisor, &decision, now_us);
            return decision;
        }
    }

    switch (supervisor->state) {
    case OBSTACLE_STATE_SENSOR_CHECK:
        count_clear(supervisor, observation,
                    supervisor->config.clear_confirm_count);
        if (supervisor->clear_count >=
            (uint8_t)supervisor->config.clear_confirm_count) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_CLEAR,
                        OBSTACLE_REASON_STARTUP_CLEAR, now_us);
        }
        break;

    case OBSTACLE_STATE_CLEAR:
        if (observation == OBSERVATION_NEAR) {
            enter_state(supervisor, &decision,
                        supervisor->config.bypass_enabled ?
                            OBSTACLE_STATE_BRAKE :
                            OBSTACLE_STATE_WAIT_CLEAR,
                        OBSTACLE_REASON_NEAR, now_us);
            decision.line_action = LINE_ACTION_SUSPEND;
        } else if (observation == OBSERVATION_UNCERTAIN) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_WAIT_CLEAR,
                        OBSTACLE_REASON_UNCERTAIN, now_us);
            decision.line_action = LINE_ACTION_SUSPEND;
        }
        break;

    case OBSTACLE_STATE_WAIT_CLEAR:
        if (observation == OBSERVATION_NEAR &&
            supervisor->config.bypass_enabled) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_BRAKE,
                        OBSTACLE_REASON_NEAR, now_us);
        } else {
            count_clear(supervisor, observation,
                        supervisor->config.clear_confirm_count);
            if (supervisor->clear_count >=
                (uint8_t)supervisor->config.clear_confirm_count) {
                enter_state(supervisor, &decision, OBSTACLE_STATE_CLEAR,
                            OBSTACLE_REASON_THREE_CLEAR, now_us);
                decision.line_action = LINE_ACTION_RESUME;
            }
        }
        break;

    case OBSTACLE_STATE_BRAKE:
        if (elapsed_us >= supervisor->config.brake_ms * 1000LL) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_STRAFE_LEFT_EDGE,
                        OBSTACLE_REASON_BRAKE_COMPLETE, now_us);
        }
        break;

    case OBSTACLE_STATE_STRAFE_LEFT_EDGE:
        if (elapsed_us >=
            supervisor->config.lateral_edge_timeout_ms * 1000LL) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_FAILSAFE,
                        OBSTACLE_REASON_TIMEOUT, now_us);
        } else if (observation == OBSERVATION_CLEAR &&
                   elapsed_us >=
                       supervisor->config.lateral_edge_min_ms * 1000LL) {
            count_clear(supervisor, observation,
                        supervisor->config.edge_clear_confirm_count);
            if (supervisor->clear_count >=
                (uint8_t)supervisor->config.edge_clear_confirm_count) {
                enter_state(supervisor, &decision,
                            OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE,
                            OBSTACLE_REASON_EDGE_CLEAR, now_us);
            }
        } else if (observation != OBSERVATION_NONE) {
            supervisor->clear_count = 0;
        }
        break;

    case OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE:
        if (observation == OBSERVATION_NEAR) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_STRAFE_LEFT_EDGE,
                        OBSTACLE_REASON_NEAR, now_us);
        } else if (elapsed_us >=
                   supervisor->config.lateral_clearance_ms * 1000LL) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_SETTLE_FORWARD,
                        OBSTACLE_REASON_SEGMENT_COMPLETE, now_us);
        }
        break;

    case OBSTACLE_STATE_SETTLE_FORWARD:
        if (elapsed_us >= supervisor->config.brake_ms * 1000LL) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_FORWARD_BYPASS,
                        OBSTACLE_REASON_BRAKE_COMPLETE, now_us);
        }
        break;

    case OBSTACLE_STATE_FORWARD_BYPASS:
        if (observation == OBSERVATION_NEAR) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_FAILSAFE,
                        OBSTACLE_REASON_NEAR, now_us);
        } else if (elapsed_us >=
                   supervisor->config.forward_bypass_ms * 1000LL) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_SETTLE_RIGHT,
                        OBSTACLE_REASON_SEGMENT_COMPLETE, now_us);
        }
        break;

    case OBSTACLE_STATE_SETTLE_RIGHT:
        if (elapsed_us >= supervisor->config.brake_ms * 1000LL) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_STRAFE_RIGHT_LINE,
                        OBSTACLE_REASON_BRAKE_COMPLETE, now_us);
            supervisor->line_search_started_us = now_us;
        }
        break;

    case OBSTACLE_STATE_STRAFE_RIGHT_LINE:
        if (line_detected(line)) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_LINE_CONFIRM,
                        OBSTACLE_REASON_LINE_SEEN, now_us);
            supervisor->clear_count = 1;
        } else if (now_us - supervisor->line_search_started_us >=
                   supervisor->config.line_search_timeout_ms * 1000LL) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_FAILSAFE,
                        OBSTACLE_REASON_TIMEOUT, now_us);
        }
        break;

    case OBSTACLE_STATE_LINE_CONFIRM:
        if (now_us - supervisor->line_search_started_us >=
            supervisor->config.line_search_timeout_ms * 1000LL) {
            enter_state(supervisor, &decision, OBSTACLE_STATE_FAILSAFE,
                        OBSTACLE_REASON_TIMEOUT, now_us);
        } else if (!line_detected(line)) {
            enter_state(supervisor, &decision,
                        OBSTACLE_STATE_STRAFE_RIGHT_LINE,
                        OBSTACLE_REASON_LINE_LOST, now_us);
        } else {
            if (supervisor->clear_count <
                (uint8_t)supervisor->config.line_confirm_count) {
                supervisor->clear_count++;
            }
            if (supervisor->clear_count >=
                (uint8_t)supervisor->config.line_confirm_count) {
                enter_state(supervisor, &decision, OBSTACLE_STATE_CLEAR,
                            OBSTACLE_REASON_LINE_CONFIRMED, now_us);
                decision.line_action = LINE_ACTION_RESUME;
            }
        }
        break;

    case OBSTACLE_STATE_FAILSAFE:
        break;
    }

    apply_policy(supervisor, &decision, now_us);
    return decision;
}

const char *obstacle_state_name(obstacle_state_t state)
{
    switch (state) {
    case OBSTACLE_STATE_SENSOR_CHECK: return "SENSOR_CHECK";
    case OBSTACLE_STATE_CLEAR: return "CLEAR";
    case OBSTACLE_STATE_WAIT_CLEAR: return "WAIT_CLEAR";
    case OBSTACLE_STATE_BRAKE: return "BRAKE";
    case OBSTACLE_STATE_STRAFE_LEFT_EDGE: return "LEFT_EDGE";
    case OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE: return "LEFT_CLEARANCE";
    case OBSTACLE_STATE_SETTLE_FORWARD: return "SETTLE_FORWARD";
    case OBSTACLE_STATE_FORWARD_BYPASS: return "FORWARD_BYPASS";
    case OBSTACLE_STATE_SETTLE_RIGHT: return "SETTLE_RIGHT";
    case OBSTACLE_STATE_STRAFE_RIGHT_LINE: return "RIGHT_LINE";
    case OBSTACLE_STATE_LINE_CONFIRM: return "LINE_CONFIRM";
    case OBSTACLE_STATE_FAILSAFE: return "FAILSAFE";
    default: return "UNKNOWN";
    }
}
