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

static void reset_drive_assist(line_follow_t *controller)
{
    controller->previous_target_a = 0;
    controller->previous_target_c = 0;
    controller->assist_a_until_us = 0;
    controller->assist_c_until_us = 0;
}

static int apply_drive_assist(line_follow_t *controller, int target,
                              int *previous_target,
                              int64_t *assist_until_us, int64_t now_us)
{
    const int previous = *previous_target;
    *previous_target = target;
    if (abs(target) < controller->config.drive_assist_threshold) {
        *assist_until_us = 0;
        return target;
    }

    const bool became_drive_wheel =
        abs(previous) < controller->config.drive_assist_threshold;
    const bool reversed = sign_of(previous) != sign_of(target);
    if (became_drive_wheel || reversed) {
        *assist_until_us = now_us +
            controller->config.drive_assist_ms * 1000LL;
    }
    if (now_us < *assist_until_us) {
        const int assisted = abs(target) >
            controller->config.drive_assist_command ? abs(target) :
            controller->config.drive_assist_command;
        return sign_of(target) * assisted;
    }
    return target;
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

void line_follow_init(line_follow_t *controller,
                      const line_follow_config_t *config)
{
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
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
}

void line_follow_suspend(line_follow_t *controller)
{
    controller->state = LINE_STATE_IDLE;
    controller->base_speed = 0;
    controller->control_error = 0;
    reset_drive_assist(controller);
}

void line_follow_resume(line_follow_t *controller)
{
    controller->lost_count = 0;
    controller->pattern_stable_count = 0;
    reset_drive_assist(controller);
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
        int a = direction * controller->config.search_speed;
        int c = -a;
        a = apply_drive_assist(controller, a,
                               &controller->previous_target_a,
                               &controller->assist_a_until_us, now_us);
        c = apply_drive_assist(controller, c,
                               &controller->previous_target_c,
                               &controller->assist_c_until_us, now_us);
        return (motor_command_t) {(int16_t)a, 0, (int16_t)c};
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
    }
    /* A single active sensor demands a tight turn. Keep its inside wheel
     * above the measured loaded dead zone. The sign checks preserve an
     * opposite direction lock while it is being confirmed. */
    if ((pattern == 0x01 || pattern == 0x02) && a > 0 &&
        a < controller->config.single_sensor_inner_command) {
        a = controller->config.single_sensor_inner_command;
    } else if ((pattern == 0x04 || pattern == 0x08) && c > 0 &&
               c < controller->config.single_sensor_inner_command) {
        c = controller->config.single_sensor_inner_command;
    }
    a = apply_drive_assist(controller, a,
                           &controller->previous_target_a,
                           &controller->assist_a_until_us, now_us);
    c = apply_drive_assist(controller, c,
                           &controller->previous_target_c,
                           &controller->assist_c_until_us, now_us);
    return (motor_command_t) {(int16_t)a, 0, (int16_t)c};
}
