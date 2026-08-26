#include "line_follow.h"

#include <stdlib.h>
#include <string.h>

static int clamp_value(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static int sign_of(int value)
{
    return value < 0 ? -1 : value > 0 ? 1 : 0;
}

static bool is_straight(uint8_t pattern)
{
    return pattern == 0x06 || pattern == 0x0f;
}

static bool is_side_only(uint8_t pattern)
{
    const bool left = (pattern & 0x0c) != 0;
    const bool right = (pattern & 0x03) != 0;
    return left != right;
}

static void reset_boost(line_follow_t *controller)
{
    controller->boost_a = (line_boost_state_t) {.armed = true};
    controller->boost_c = (line_boost_state_t) {.armed = true};
    controller->boost_a_active = false;
    controller->boost_c_active = false;
}

static int apply_boost(const line_follow_config_t *config, int command,
                       line_boost_state_t *state, bool *active,
                       int64_t now_us)
{
    const int direction = sign_of(command);
    if (direction == 0) {
        if (state->last_direction != 0) {
            state->zero_since_us = now_us;
        }
        state->last_direction = 0;
        state->boost_until_us = 0;
        if (!state->armed && state->zero_since_us > 0 &&
            now_us - state->zero_since_us >=
                config->boost_rearm_ms * 1000LL) {
            state->armed = true;
        }
        *active = false;
        return 0;
    }
    if (state->last_direction != 0 && direction != state->last_direction) {
        state->armed = true;
    } else if (state->last_direction == 0 && !state->armed &&
               state->zero_since_us > 0 &&
               now_us - state->zero_since_us >=
                   config->boost_rearm_ms * 1000LL) {
        state->armed = true;
    }
    if (state->armed) {
        state->boost_until_us = now_us + config->boost_ms * 1000LL;
        state->armed = false;
    }
    state->last_direction = direction;
    if (now_us < state->boost_until_us &&
        abs(command) < config->boost_command) {
        *active = true;
        return direction * config->boost_command;
    }
    *active = false;
    return command;
}

static void update_direction_lock(line_follow_t *controller, int direction)
{
    if (direction == 0) {
        return;
    }
    if (controller->locked_direction == 0) {
        if (controller->direction_candidate != direction) {
            controller->direction_candidate = direction;
            controller->direction_candidate_count = 1;
        } else if (controller->direction_candidate_count <
                   (unsigned)controller->config.direction_confirm_count) {
            controller->direction_candidate_count++;
        }
        if (controller->direction_candidate_count >=
            (unsigned)controller->config.direction_confirm_count) {
            controller->locked_direction = direction;
            controller->direction_candidate = 0;
            controller->direction_candidate_count = 0;
        }
        return;
    }
    if (direction == controller->locked_direction) {
        controller->direction_candidate = 0;
        controller->direction_candidate_count = 0;
        return;
    }
    if (controller->direction_candidate != direction) {
        controller->direction_candidate = direction;
        controller->direction_candidate_count = 1;
    } else if (controller->direction_candidate_count <
               (unsigned)controller->config.direction_confirm_count) {
        controller->direction_candidate_count++;
    }
    if (controller->direction_candidate_count >=
        (unsigned)controller->config.direction_confirm_count) {
        controller->locked_direction = direction;
        controller->direction_candidate = 0;
        controller->direction_candidate_count = 0;
    }
}

static motor_command_t boosted(line_follow_t *controller, int a, int c,
                               int64_t now_us)
{
    return (motor_command_t) {
        .a = (int16_t)apply_boost(&controller->config, a,
                                  &controller->boost_a,
                                  &controller->boost_a_active, now_us),
        .b = 0,
        .c = (int16_t)apply_boost(&controller->config, c,
                                  &controller->boost_c,
                                  &controller->boost_c_active, now_us),
    };
}

void line_follow_init(line_follow_t *controller,
                      const line_follow_config_t *config)
{
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    reset_boost(controller);
}

void line_follow_reset_for_start(line_follow_t *controller,
                                 line_sensor_sample_t initial)
{
    const line_follow_config_t config = controller->config;
    memset(controller, 0, sizeof(*controller));
    controller->config = config;
    controller->sensors = initial;
    controller->pattern = line_sensor_pattern(initial);
    controller->previous_pattern = controller->pattern;
    controller->state = LINE_STATE_IDLE;
    reset_boost(controller);
}

void line_follow_suspend(line_follow_t *controller)
{
    reset_boost(controller);
    controller->state = LINE_STATE_IDLE;
    controller->base_speed = 0;
    controller->control_error = 0;
}

void line_follow_resume(line_follow_t *controller)
{
    controller->lost_count = 0;
    controller->pattern_stable_count = 0;
}

motor_command_t line_follow_step(line_follow_t *controller,
                                 line_sensor_sample_t sensors,
                                 int speed_ceiling, int64_t now_us)
{
    const uint8_t pattern = line_sensor_pattern(sensors);
    const int active_count = sensors.left + sensors.left_center +
                             sensors.right_center + sensors.right;
    controller->sensors = sensors;
    controller->active_count = active_count;
    controller->pattern = pattern;
    if (pattern == controller->previous_pattern) {
        if (controller->pattern_stable_count < 255) {
            controller->pattern_stable_count++;
        }
    } else {
        controller->previous_pattern = pattern;
        controller->pattern_stable_count = 1;
    }
    if (active_count == 0) {
        controller->lost_count++;
        controller->error = 0;
        controller->control_error = 0;
        controller->base_speed = 0;
        const int direction = controller->locked_direction == 0 ? -1 :
                              controller->locked_direction;
        controller->state = direction < 0 ? LINE_STATE_SEARCH_LEFT :
                                            LINE_STATE_SEARCH_RIGHT;
        return direction < 0 ?
            boosted(controller, -controller->config.search_speed,
                    controller->config.search_speed, now_us) :
            boosted(controller, controller->config.search_speed,
                    -controller->config.search_speed, now_us);
    }

    controller->lost_count = 0;
    const int weighted = sensors.left * -6 + sensors.left_center * -2 +
                         sensors.right_center * 2 + sensors.right * 6;
    const int error = weighted / active_count;
    const int direction = sign_of(error);
    update_direction_lock(controller, direction);
    controller->error = error;
    if (error != 0) {
        controller->last_error = error;
    }
    const bool straight = is_straight(pattern);
    const bool edge = is_side_only(pattern);
    int base = straight ? controller->config.straight_speed :
               edge ? controller->config.edge_speed :
                      controller->config.curve_speed;
    if (base > speed_ceiling) {
        base = speed_ceiling;
    }
    controller->state = straight ? LINE_STATE_STRAIGHT : LINE_STATE_CURVE;
    controller->base_speed = base;
    int control_error = error;
    if (controller->locked_direction != 0 && direction != 0 &&
        direction != controller->locked_direction &&
        controller->direction_candidate_count <
            (unsigned)controller->config.direction_confirm_count) {
        control_error = controller->locked_direction *
            (abs(error) > controller->config.direction_hold_error ?
             abs(error) : controller->config.direction_hold_error);
    }
    controller->control_error = control_error;
    const int correction = clamp_value(control_error * controller->config.kp,
                                       -controller->config.max_correction,
                                       controller->config.max_correction);
    int a = -base + correction;
    int c = -base - correction;
    if (!straight) {
        const int limit = edge ? controller->config.edge_max :
                                 controller->config.curve_max;
        const int maximum = abs(a) > abs(c) ? abs(a) : abs(c);
        if (maximum > limit) {
            a = a * limit / maximum;
            c = c * limit / maximum;
        }
        if (a != 0 && abs(a) < controller->config.minimum_active) {
            a = sign_of(a) * controller->config.minimum_active;
        }
        if (c != 0 && abs(c) < controller->config.minimum_active) {
            c = sign_of(c) * controller->config.minimum_active;
        }
    }
    return boosted(controller, a, c, now_us);
}
