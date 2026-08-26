#pragma once

#include "config_types.h"
#include "motion_types.h"

/*
 * Inverse kinematics for the experimentally identified chassis layout:
 * A = right wheel, B = rear wheel, C = left wheel.
 *
 * The forward and clockwise bases are physically verified.  The lateral
 * basis follows from the 120-degree kiwi geometry and must be confirmed by
 * the bounded q/e tests before automatic bypass is enabled.
 */
motor_command_t kiwi_inverse_kinematics(
    body_motion_command_t motion,
    const kiwi_kinematics_config_t *config);
