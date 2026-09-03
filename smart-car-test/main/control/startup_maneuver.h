#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_types.h"
#include "motion_types.h"

typedef enum {
    STARTUP_MANEUVER_WAIT_FOR_CLEAR,
    STARTUP_MANEUVER_FORWARD,
    STARTUP_MANEUVER_SETTLE_AFTER_FORWARD,
    STARTUP_MANEUVER_TURN_RIGHT,
    STARTUP_MANEUVER_SETTLE_AFTER_TURN,
    STARTUP_MANEUVER_COMPLETE,
} startup_maneuver_phase_t;

typedef enum {
    STARTUP_MANEUVER_TRANSITION_NONE,
    STARTUP_MANEUVER_TRANSITION_TO_FORWARD,
    STARTUP_MANEUVER_TRANSITION_TO_SETTLE_AFTER_FORWARD,
    STARTUP_MANEUVER_TRANSITION_TO_TURN_RIGHT,
    STARTUP_MANEUVER_TRANSITION_TO_SETTLE_AFTER_TURN,
    STARTUP_MANEUVER_TRANSITION_TO_COMPLETE,
} startup_maneuver_transition_t;

typedef struct {
    startup_maneuver_config_t config;
    startup_maneuver_phase_t phase;
    int64_t phase_started_us;
} startup_maneuver_t;

typedef struct {
    body_motion_command_t motion;
    startup_maneuver_transition_t transition;
    bool complete;
} startup_maneuver_decision_t;

void startup_maneuver_init(startup_maneuver_t *maneuver,
                           const startup_maneuver_config_t *config);
void startup_maneuver_reset(startup_maneuver_t *maneuver);
startup_maneuver_decision_t startup_maneuver_step(
    startup_maneuver_t *maneuver, int64_t now_us);
bool startup_maneuver_is_complete(const startup_maneuver_t *maneuver);
const char *startup_maneuver_phase_name(startup_maneuver_phase_t phase);
