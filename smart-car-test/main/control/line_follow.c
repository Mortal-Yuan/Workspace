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

static int choose_search_direction(const line_follow_t *controller)
{
    if (controller->locked_direction != 0) {
        return controller->locked_direction;
    }
    if (controller->last_steering_direction != 0) {
        return controller->last_steering_direction;
    }
    if (controller->last_error != 0) {
        return sign_of(controller->last_error);
    }
    /* A deterministic default is preferable to remaining stationary when
     * autonomy was deliberately started without a direction history. */
    return -1;
}

static void clear_search_state(line_follow_t *controller)
{
    controller->lost_count = 0;
    controller->lost_since_us = 0;
    controller->search_direction = 0;
    controller->reacquire_count = 0;
    controller->last_reacquire_frame_seq = 0;
}

static void reset_drive_assist(line_follow_t *controller)
{
    controller->previous_target_a = 0;
    controller->previous_target_c = 0;
    controller->assist_a_until_us = 0;
    controller->assist_c_until_us = 0;
}

static void reset_motion_memory(line_follow_t *controller)
{
    controller->last_cruise_command = motor_command_zero();
    controller->lost_memory_command = motor_command_zero();
    controller->last_cruise_valid = false;
    controller->lost_memory_valid = false;
    controller->turn_memory_direction = 0;
    controller->pending_turn_direction = 0;
    controller->pending_turn_since_us = 0;
}

static motor_command_t apply_turn_memory(line_follow_t *controller,
                                         motor_command_t desired,
                                         bool steering_valid,
                                         int steering_permille,
                                         int64_t now_us)
{
    if (!steering_valid) {
        controller->turn_memory_direction = 0;
        controller->pending_turn_direction = 0;
        controller->pending_turn_since_us = 0;
        return desired;
    }

    const int magnitude = abs(steering_permille);
    if (magnitude <= controller->config.turn_memory_release_permille) {
        controller->turn_memory_direction = 0;
        controller->pending_turn_direction = 0;
        controller->pending_turn_since_us = 0;
        return desired;
    }
    if (magnitude < controller->config.turn_memory_threshold_permille) {
        controller->pending_turn_direction = 0;
        controller->pending_turn_since_us = 0;
        return desired;
    }

    const int direction = sign_of(steering_permille);
    if (direction == controller->turn_memory_direction) {
        controller->pending_turn_direction = 0;
        controller->pending_turn_since_us = 0;
        return desired;
    }
    if (!controller->last_cruise_valid) {
        controller->turn_memory_direction = direction;
        return desired;
    }
    if (controller->pending_turn_direction != direction) {
        controller->pending_turn_direction = direction;
        controller->pending_turn_since_us = now_us;
    }
    if (now_us - controller->pending_turn_since_us <
        controller->config.turn_memory_ms * 1000LL) {
        return controller->last_cruise_command;
    }

    controller->turn_memory_direction = direction;
    controller->pending_turn_direction = 0;
    controller->pending_turn_since_us = 0;
    return desired;
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
    reset_motion_memory(controller);
}

void line_follow_resume(line_follow_t *controller)
{
    clear_search_state(controller);
    controller->pattern_stable_count = 0;
    reset_drive_assist(controller);
    reset_motion_memory(controller);
}

motor_command_t line_follow_step_camera(line_follow_t *controller,
                                        line_sensor_sample_t sensors,
                                        bool steering_valid,
                                        int steering_permille,
                                        uint32_t frame_seq,
                                        int speed_ceiling,
                                        int64_t now_us)
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
        if (controller->lost_count == 0) {
            controller->lost_since_us = now_us;
            controller->search_direction =
                choose_search_direction(controller);
            controller->reacquire_count = 0;
            controller->last_reacquire_frame_seq = 0;
            controller->lost_memory_command =
                controller->last_cruise_command;
            controller->lost_memory_valid =
                controller->last_cruise_valid;
            controller->last_cruise_valid = false;
            controller->turn_memory_direction = 0;
            controller->pending_turn_direction = 0;
            controller->pending_turn_since_us = 0;
            reset_drive_assist(controller);
        }
        if (controller->lost_count < UINT32_MAX) {
            controller->lost_count++;
        }
        controller->reacquire_count = 0;
        controller->last_reacquire_frame_seq = 0;
        controller->error = 0;
        controller->control_error = 0;
        controller->base_speed = 0;
        controller->steering_permille = 0;
        const int64_t elapsed_us = now_us - controller->lost_since_us;
        controller->state = controller->search_direction < 0 ?
            LINE_STATE_SEARCH_LEFT : LINE_STATE_SEARCH_RIGHT;
        if (controller->lost_memory_valid &&
            elapsed_us <
                controller->config.lost_motion_memory_ms * 1000LL) {
            return controller->lost_memory_command;
        }
        if (elapsed_us <
            controller->config.lost_search_delay_ms * 1000LL) {
            reset_drive_assist(controller);
            return motor_command_zero();
        }

        const int64_t search_elapsed_us = elapsed_us -
            controller->config.lost_search_delay_ms * 1000LL;
        const int64_t initial_leg_us =
            controller->config.search_primary_ms * 1000LL;
        const int64_t leg_increment_us =
            (controller->config.search_reverse_ms -
             controller->config.search_primary_ms) * 1000LL;
        const int64_t maximum_leg_us =
            controller->config.search_max_sweep_ms * 1000LL;
        /* Expand successive alternating legs from 1.2 through 7.2 seconds.
         * Their cumulative headings reach +/-1.2, +/-2.4, and +/-3.6 second
         * equivalents on the two sides.  Once maximum width is reached,
         * retain full-width legs instead of expanding without bound. */
        int64_t remaining_us = search_elapsed_us;
        int64_t leg_duration_us = initial_leg_us;
        uint64_t leg_index = 0;
        while (leg_duration_us < maximum_leg_us &&
               remaining_us >= leg_duration_us) {
            remaining_us -= leg_duration_us;
            ++leg_index;
            const int64_t expanded_us = initial_leg_us +
                (int64_t)leg_index * leg_increment_us;
            leg_duration_us = expanded_us < maximum_leg_us ?
                expanded_us : maximum_leg_us;
        }
        if (leg_duration_us == maximum_leg_us &&
            remaining_us >= maximum_leg_us) {
            leg_index += (uint64_t)(remaining_us / maximum_leg_us);
        }
        const int direction = (leg_index & 1U) == 0 ?
            controller->search_direction : -controller->search_direction;
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

    if (controller->lost_count != 0) {
        controller->state = LINE_STATE_REACQUIRE;
        controller->base_speed = 0;
        controller->error = 0;
        controller->control_error = 0;
        reset_drive_assist(controller);
        if (frame_seq != 0 &&
            frame_seq != controller->last_reacquire_frame_seq) {
            controller->last_reacquire_frame_seq = frame_seq;
            if (controller->reacquire_count <
                (unsigned)controller->config.search_reacquire_frames) {
                controller->reacquire_count++;
            }
        }
        if (controller->reacquire_count <
            (unsigned)controller->config.search_reacquire_frames) {
            return motor_command_zero();
        }
        clear_search_state(controller);
        controller->lost_memory_valid = false;
    }

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
    controller->steering_permille = steering_valid ?
        clamp_value(steering_permille, -1000, 1000) : 0;
    if (steering_valid &&
        abs(controller->steering_permille) >=
            controller->config.search_direction_threshold_permille) {
        controller->last_steering_direction =
            sign_of(controller->steering_permille);
    }
    const bool holding_opposite_direction =
        controller->locked_direction != 0 && direction != 0 &&
        direction != controller->locked_direction &&
        controller->direction_candidate_count <
            (unsigned)controller->config.direction_confirm_count;
    int correction;
    if (steering_valid && !holding_opposite_direction) {
        const int numerator = controller->steering_permille *
            controller->config.max_correction;
        correction = numerator >= 0 ?
            (numerator + 300) / 600 : (numerator - 300) / 600;
        correction = clamp_value(correction,
                                 -controller->config.max_correction,
                                 controller->config.max_correction);
    } else {
        correction = clamp_value(control_error * controller->config.kp,
                                 -controller->config.max_correction,
                                 controller->config.max_correction);
    }
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
    motor_command_t cruise = {(int16_t)a, 0, (int16_t)c};
    cruise = apply_turn_memory(
        controller, cruise, steering_valid,
        controller->steering_permille, now_us);
    controller->last_cruise_command = cruise;
    controller->last_cruise_valid = true;
    a = cruise.a;
    c = cruise.c;
    a = apply_drive_assist(controller, a,
                           &controller->previous_target_a,
                           &controller->assist_a_until_us, now_us);
    c = apply_drive_assist(controller, c,
                           &controller->previous_target_c,
                           &controller->assist_c_until_us, now_us);
    return (motor_command_t) {(int16_t)a, 0, (int16_t)c};
}

motor_command_t line_follow_step(line_follow_t *controller,
                                 line_sensor_sample_t sensors,
                                 int speed_ceiling, int64_t now_us)
{
    controller->synthetic_frame_seq++;
    if (controller->synthetic_frame_seq == 0) {
        controller->synthetic_frame_seq = 1;
    }
    return line_follow_step_camera(
        controller, sensors, false, 0,
        controller->synthetic_frame_seq, speed_ceiling, now_us);
}
