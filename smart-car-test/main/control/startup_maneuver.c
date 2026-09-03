#include "startup_maneuver.h"

#include <string.h>

static void enter_phase(startup_maneuver_t *maneuver,
                        startup_maneuver_decision_t *decision,
                        startup_maneuver_phase_t phase,
                        startup_maneuver_transition_t transition,
                        int64_t now_us)
{
    maneuver->phase = phase;
    maneuver->phase_started_us = now_us;
    decision->transition = transition;
}

void startup_maneuver_init(startup_maneuver_t *maneuver,
                           const startup_maneuver_config_t *config)
{
    memset(maneuver, 0, sizeof(*maneuver));
    maneuver->config = *config;
    maneuver->phase = STARTUP_MANEUVER_WAIT_FOR_CLEAR;
}

void startup_maneuver_reset(startup_maneuver_t *maneuver)
{
    maneuver->phase = STARTUP_MANEUVER_WAIT_FOR_CLEAR;
    maneuver->phase_started_us = 0;
}

startup_maneuver_decision_t startup_maneuver_step(
    startup_maneuver_t *maneuver, int64_t now_us)
{
    startup_maneuver_decision_t decision = {
        .motion = body_motion_zero(),
    };
    int64_t elapsed_us = now_us - maneuver->phase_started_us;

    switch (maneuver->phase) {
    case STARTUP_MANEUVER_WAIT_FOR_CLEAR:
        enter_phase(maneuver, &decision, STARTUP_MANEUVER_FORWARD,
                    STARTUP_MANEUVER_TRANSITION_TO_FORWARD, now_us);
        decision.motion.forward =
            (int16_t)maneuver->config.forward_start_speed;
        break;

    case STARTUP_MANEUVER_FORWARD:
        if (elapsed_us >= maneuver->config.forward_ms * 1000LL) {
            enter_phase(
                maneuver, &decision,
                STARTUP_MANEUVER_SETTLE_AFTER_FORWARD,
                STARTUP_MANEUVER_TRANSITION_TO_SETTLE_AFTER_FORWARD,
                now_us);
        } else {
            decision.motion.forward = (int16_t)(
                elapsed_us < maneuver->config.forward_boost_ms * 1000LL ?
                maneuver->config.forward_start_speed :
                maneuver->config.forward_speed);
        }
        break;

    case STARTUP_MANEUVER_SETTLE_AFTER_FORWARD:
        if (elapsed_us >= maneuver->config.settle_ms * 1000LL) {
            enter_phase(maneuver, &decision,
                        STARTUP_MANEUVER_TURN_RIGHT,
                        STARTUP_MANEUVER_TRANSITION_TO_TURN_RIGHT,
                        now_us);
            decision.motion.clockwise =
                (int16_t)maneuver->config.right_turn_speed;
        }
        break;

    case STARTUP_MANEUVER_TURN_RIGHT:
        if (elapsed_us >= maneuver->config.right_turn_ms * 1000LL) {
            enter_phase(
                maneuver, &decision,
                STARTUP_MANEUVER_SETTLE_AFTER_TURN,
                STARTUP_MANEUVER_TRANSITION_TO_SETTLE_AFTER_TURN,
                now_us);
        } else {
            decision.motion.clockwise =
                (int16_t)maneuver->config.right_turn_speed;
        }
        break;

    case STARTUP_MANEUVER_SETTLE_AFTER_TURN:
        if (elapsed_us >= maneuver->config.settle_ms * 1000LL) {
            enter_phase(maneuver, &decision,
                        STARTUP_MANEUVER_COMPLETE,
                        STARTUP_MANEUVER_TRANSITION_TO_COMPLETE,
                        now_us);
            decision.complete = true;
        }
        break;

    case STARTUP_MANEUVER_COMPLETE:
        decision.complete = true;
        break;
    }

    return decision;
}

bool startup_maneuver_is_complete(const startup_maneuver_t *maneuver)
{
    return maneuver->phase == STARTUP_MANEUVER_COMPLETE;
}

const char *startup_maneuver_phase_name(startup_maneuver_phase_t phase)
{
    switch (phase) {
    case STARTUP_MANEUVER_WAIT_FOR_CLEAR: return "startup_wait_clear";
    case STARTUP_MANEUVER_FORWARD: return "startup_forward_20cm";
    case STARTUP_MANEUVER_SETTLE_AFTER_FORWARD:
        return "startup_settle_forward";
    case STARTUP_MANEUVER_TURN_RIGHT: return "startup_turn_right_60deg";
    case STARTUP_MANEUVER_SETTLE_AFTER_TURN:
        return "startup_settle_turn";
    case STARTUP_MANEUVER_COMPLETE: return "startup_line_follow";
    default: return "startup_unknown";
    }
}
