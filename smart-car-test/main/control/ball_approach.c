#include "ball_approach.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "kiwi_kinematics.h"

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static bool is_red_ball(const camera_ball_observation_t *ball)
{
    return ball->detected && ball->color == BALL_COLOR_RED;
}

static bool is_blue_goal(const camera_ball_observation_t *goal)
{
    return goal->detected && goal->color == BALL_COLOR_BLUE;
}

static bool is_blue_candidate(const camera_ball_observation_t *goal)
{
    return goal->candidate && goal->color == BALL_COLOR_BLUE;
}

static int goal_track_distance(const camera_ball_observation_t *goal,
                               const camera_ball_observation_t *track)
{
    return abs(goal->center_x_permille - track->center_x_permille) +
        abs((int)goal->center_y_permille - (int)track->center_y_permille);
}

static bool first_goal_preferred(const camera_ball_observation_t *first,
                                 const camera_ball_observation_t *second)
{
    if (first->detected != second->detected) return first->detected;
    if (first->stable_frames != second->stable_frames) {
        return first->stable_frames > second->stable_frames;
    }
    if (first->confidence_permille != second->confidence_permille) {
        return first->confidence_permille > second->confidence_permille;
    }
    return first->matched_pixels >= second->matched_pixels;
}

/* The camera still reports up to two positional blue components for the
 * monitor, but ball control treats them as an unordered set.  The first
 * confirmed component seen after start becomes the preference; afterwards
 * nearest-frame continuity keeps that physical target selected as it crosses
 * the image center. */
static void update_selected_goal(ball_approach_t *controller,
                                 const camera_line_snapshot_t *camera)
{
    const camera_ball_observation_t *first = &camera->left_target;
    const camera_ball_observation_t *second = &camera->right_target;
    const bool first_visible = is_blue_candidate(first);
    const bool second_visible = is_blue_candidate(second);
    const camera_ball_observation_t previous = controller->selected_goal;
    const camera_ball_observation_t *selected = NULL;

    if (first_visible && second_visible) {
        if (controller->goal_preference_locked) {
            selected = goal_track_distance(first, &previous) <=
                    goal_track_distance(second, &previous) ? first : second;
        } else {
            selected = first_goal_preferred(first, second) ? first : second;
        }
    } else if (first_visible) {
        selected = first;
    } else if (second_visible) {
        selected = second;
    }

    if (selected != NULL) {
        controller->selected_goal = *selected;
        if (selected->detected) controller->goal_preference_locked = true;
    } else {
        /* Preserve the last position for reacquisition while making the
         * current observation unavailable to motion control. */
        controller->selected_goal.candidate = false;
        controller->selected_goal.detected = false;
        controller->selected_goal.stable_frames = 0;
    }
}

static motor_command_t turn_command(int direction, int speed)
{
    return (motor_command_t) {
        .a = (int16_t)(direction * speed),
        .b = 0,
        .c = (int16_t)(-direction * speed),
    };
}

static motor_command_t route_shift_command(
    const ball_approach_t *controller, int direction)
{
    return kiwi_inverse_kinematics(
        (body_motion_command_t) {
            .left = (int16_t)(direction *
                controller->config.route_lateral_speed),
        },
        &controller->kinematics_config);
}

static int ball_center_error(const ball_approach_t *controller,
                             const camera_ball_observation_t *ball)
{
    return ball->center_x_permille -
        controller->config.target_center_x_permille;
}

static int goal_center_error(const ball_approach_t *controller,
                             const camera_ball_observation_t *goal)
{
    return goal->center_x_permille -
        controller->config.target_center_x_permille;
}

/* A negative value means that the red ball is left of the blue goal. */
static int route_error(const camera_ball_observation_t *ball,
                       const camera_ball_observation_t *goal)
{
    return ball->center_x_permille - goal->center_x_permille;
}

static bool ball_is_inside_clip(
    const ball_approach_t *controller,
    const camera_ball_observation_t *ball, int error)
{
    const bool large_enough =
        ball->height_permille >=
            controller->config.capture_height_permille ||
        ball->area_permille >= controller->config.capture_area_permille;
    return is_red_ball(ball) &&
        abs(error) <= controller->config.realign_threshold_permille &&
        ball->center_y_permille >=
            controller->config.capture_center_y_permille &&
        ball->box_bottom_permille >=
            controller->config.capture_box_bottom_permille &&
        large_enough;
}

static bool ball_is_in_goal(
    const ball_approach_t *controller,
    const camera_ball_observation_t *ball,
    const camera_ball_observation_t *goal)
{
    if (!is_red_ball(ball) || !is_blue_goal(goal)) {
        return false;
    }
    const int ball_x = ((int)ball->center_x_permille + 1000) / 2;
    const int ball_y = ball->center_y_permille;
    const int margin = controller->config.goal_overlap_margin_permille;
    return ball_x >= (int)goal->box_left_permille - margin &&
        ball_x <= (int)goal->box_right_permille + margin &&
        ball_y >= (int)goal->box_top_permille - margin &&
        ball_y <= (int)goal->box_bottom_permille + margin;
}

static motor_command_t steered_forward_command(
    int speed, int error, int steering_gain_permille,
    int maximum_correction)
{
    const int correction = clamp_int(
        error * steering_gain_permille / 1000,
        -maximum_correction, maximum_correction);
    return (motor_command_t) {
        .a = (int16_t)(-speed + correction),
        .b = 0,
        .c = (int16_t)(-speed - correction),
    };
}

static motor_command_t approach_command(
    const ball_approach_t *controller,
    const camera_ball_observation_t *ball, int error, int64_t now_us)
{
    int speed = controller->config.far_forward_speed;
    if (ball->center_y_permille >= controller->config.near_y_permille) {
        speed = controller->config.near_forward_speed;
    } else if (ball->center_y_permille >=
               controller->config.medium_y_permille) {
        speed = controller->config.medium_forward_speed;
    }
    if (now_us - controller->approach_started_us <
        controller->config.forward_boost_ms * 1000LL &&
        speed < controller->config.forward_boost_speed) {
        speed = controller->config.forward_boost_speed;
    }
    return steered_forward_command(
        speed, error, controller->config.steering_gain_permille,
        controller->config.maximum_correction);
}

static motor_command_t push_command(
    const ball_approach_t *controller, int goal_error, int64_t now_us)
{
    int speed = controller->config.push_speed;
    if (controller->push_motion_started_us != 0 &&
        now_us - controller->push_motion_started_us <
            controller->config.push_boost_ms * 1000LL) {
        speed = controller->config.push_boost_speed;
    }
    return steered_forward_command(
        speed, goal_error,
        controller->config.push_steering_gain_permille,
        controller->config.push_maximum_correction);
}

static void enter_state(ball_approach_t *controller,
                        ball_approach_decision_t *decision,
                        ball_approach_state_t state,
                        ball_approach_reason_t reason,
                        int64_t now_us)
{
    if (controller->state == state) return;
    controller->state = state;
    controller->last_reason = reason;
    controller->state_started_us = now_us;
    controller->held_command = motor_command_zero();
    decision->transitioned = true;
    decision->reason = reason;
}

static void begin_route_alignment(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    int64_t now_us)
{
    controller->align_frames = 0;
    controller->route_started_us = now_us;
    controller->route_completed = false;
    enter_state(controller, decision, BALL_APPROACH_STATE_ROUTE_ALIGN,
                BALL_APPROACH_REASON_CONFIRMED, now_us);
}

static void after_red_confirmed(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    const camera_ball_observation_t *goal, int64_t now_us)
{
    controller->search_motion_ms = 0;
    if (is_blue_goal(goal)) {
        begin_route_alignment(controller, decision, now_us);
    } else {
        /* A distant blue target is optional at this stage.  Follow the red
         * ball first; target search is deferred until capture is confirmed. */
        controller->align_frames = 0;
        controller->route_completed = false;
        enter_state(controller, decision, BALL_APPROACH_STATE_ALIGN,
                    BALL_APPROACH_REASON_GOAL_MISSING, now_us);
    }
}

static void begin_goal_search(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    bool settle_first, int64_t now_us)
{
    controller->align_frames = 0;
    controller->search_motion_ms = 0;
    controller->settle_next_state = BALL_APPROACH_STATE_GOAL_SEARCH_LEFT;
    enter_state(controller, decision,
                settle_first ? BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE :
                               BALL_APPROACH_STATE_GOAL_SEARCH_LEFT,
                BALL_APPROACH_REASON_GOAL_MISSING, now_us);
}

static void begin_route_alignment_after_settle(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    int64_t now_us)
{
    controller->align_frames = 0;
    controller->route_started_us = now_us;
    controller->route_completed = false;
    enter_state(controller, decision, BALL_APPROACH_STATE_ROUTE_SETTLE,
                BALL_APPROACH_REASON_CONFIRMED, now_us);
}

static void after_capture_confirmed(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    const camera_ball_observation_t *goal, int64_t now_us)
{
    controller->align_frames = 0;
    controller->goal_frames = 0;
    controller->push_started_us = now_us;
    if (is_blue_goal(goal)) {
        enter_state(controller, decision, BALL_APPROACH_STATE_PUSH_ALIGN,
                    BALL_APPROACH_REASON_CAPTURE_CONFIRMED, now_us);
    } else {
        begin_goal_search(controller, decision, false, now_us);
    }
}

static void stop_for_unconfirmed_red(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    const camera_ball_observation_t *ball, int64_t now_us)
{
    controller->align_frames = 0;
    controller->route_completed = false;
    controller->approach_started_us = 0;
    controller->search_motion_ms = 0;
    if (ball->candidate && ball->color == BALL_COLOR_RED) {
        enter_state(controller, decision,
                    BALL_APPROACH_STATE_ACQUIRE_STOP,
                    BALL_APPROACH_REASON_LOST, now_us);
    } else {
        controller->settle_next_state = BALL_APPROACH_STATE_SEARCH_LEFT;
        enter_state(controller, decision,
                    BALL_APPROACH_STATE_SEARCH_SETTLE,
                    BALL_APPROACH_REASON_LOST, now_us);
    }
}

static void enter_anomaly_wait(
    ball_approach_t *controller, ball_approach_decision_t *decision,
    ball_approach_reason_t reason, int64_t now_us)
{
    enter_state(controller, decision, BALL_APPROACH_STATE_RECOVERY_WAIT,
                reason, now_us);
}

void ball_approach_init(ball_approach_t *controller,
                        const ball_approach_config_t *config,
                        const kiwi_kinematics_config_t *kinematics_config)
{
    if (controller == NULL || config == NULL || kinematics_config == NULL) {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->kinematics_config = *kinematics_config;
    controller->state = BALL_APPROACH_STATE_IDLE;
    controller->initialized = true;
}

void ball_approach_reset(ball_approach_t *controller)
{
    const ball_approach_config_t config = controller->config;
    const kiwi_kinematics_config_t kinematics_config =
        controller->kinematics_config;
    const bool initialized = controller->initialized;
    memset(controller, 0, sizeof(*controller));
    controller->config = config;
    controller->kinematics_config = kinematics_config;
    controller->state = BALL_APPROACH_STATE_IDLE;
    controller->initialized = initialized;
}

void ball_approach_start(ball_approach_t *controller, int64_t now_us)
{
    ball_approach_reset(controller);
    controller->state = BALL_APPROACH_STATE_SEARCH_LEFT;
    controller->last_reason = BALL_APPROACH_REASON_STARTED;
    controller->run_started_us = now_us;
    controller->state_started_us = now_us;
}

static bool is_route_state(ball_approach_state_t state)
{
    return state == BALL_APPROACH_STATE_ROUTE_ALIGN ||
        state == BALL_APPROACH_STATE_ROUTE_SHIFT ||
        state == BALL_APPROACH_STATE_ROUTE_SETTLE;
}

static bool is_push_state(ball_approach_state_t state)
{
    return state == BALL_APPROACH_STATE_PUSH_ALIGN ||
        state == BALL_APPROACH_STATE_PUSH_ALIGN_PULSE ||
        state == BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE ||
        state == BALL_APPROACH_STATE_PUSH ||
        state == BALL_APPROACH_STATE_GOAL_VERIFY;
}

static bool is_motion_state(ball_approach_state_t state)
{
    return state == BALL_APPROACH_STATE_SEARCH_LEFT ||
        state == BALL_APPROACH_STATE_SEARCH_RIGHT ||
        state == BALL_APPROACH_STATE_ROUTE_SHIFT ||
        state == BALL_APPROACH_STATE_ALIGN_PULSE ||
        state == BALL_APPROACH_STATE_APPROACH ||
        state == BALL_APPROACH_STATE_GOAL_SEARCH_LEFT ||
        state == BALL_APPROACH_STATE_GOAL_SEARCH_RIGHT ||
        state == BALL_APPROACH_STATE_PUSH_ALIGN_PULSE ||
        state == BALL_APPROACH_STATE_PUSH;
}

ball_approach_decision_t ball_approach_step(
    ball_approach_t *controller, const camera_line_snapshot_t *camera,
    int64_t now_us)
{
    ball_approach_decision_t decision = {
        .command = motor_command_zero(),
        .state = controller != NULL ? controller->state :
                                      BALL_APPROACH_STATE_FAILSAFE,
    };
    if (controller == NULL || camera == NULL || !controller->initialized) {
        decision.failsafe = true;
        return decision;
    }
    if (controller->state == BALL_APPROACH_STATE_IDLE) return decision;

    const camera_ball_observation_t *ball = &camera->ball;
    const bool new_frame = camera->decoded_frames !=
        controller->last_frame_seq;
    if (new_frame) {
        controller->last_frame_seq = camera->decoded_frames;
        update_selected_goal(controller, camera);
    }
    const camera_ball_observation_t *goal = &controller->selected_goal;
    const int red_error = ball_center_error(controller, ball);
    const int blue_error = goal_center_error(controller, goal);
    const int red_blue_error = route_error(ball, goal);

    int reported_error = red_error;
    if (is_route_state(controller->state)) {
        reported_error = red_blue_error;
    } else if (is_push_state(controller->state)) {
        reported_error = blue_error;
    }
    decision.center_error_permille = (int16_t)clamp_int(
        reported_error, -2000, 2000);
    controller->last_error_permille = decision.center_error_permille;

    if (controller->state != BALL_APPROACH_STATE_DONE &&
        controller->state != BALL_APPROACH_STATE_FAILSAFE &&
        controller->state != BALL_APPROACH_STATE_RECOVERY_WAIT) {
        if (!camera->fresh) {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_CAMERA_STALE, now_us);
        } else if (now_us - controller->run_started_us >=
                   controller->config.maximum_total_ms * 1000LL) {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_TOTAL_TIMEOUT, now_us);
        } else if (is_route_state(controller->state) &&
                   now_us - controller->route_started_us >=
                       controller->config.route_align_timeout_ms * 1000LL) {
            controller->align_frames = 0;
            controller->route_completed = true;
            enter_state(controller, &decision, BALL_APPROACH_STATE_ALIGN,
                        BALL_APPROACH_REASON_ROUTE_BEST_EFFORT, now_us);
        } else if (is_push_state(controller->state) &&
                   now_us - controller->push_started_us >=
                       controller->config.push_timeout_ms * 1000LL) {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_PUSH_TIMEOUT, now_us);
        }
    }

    const int64_t elapsed_us = now_us - controller->state_started_us;
    switch (controller->state) {
    case BALL_APPROACH_STATE_SEARCH_LEFT:
    case BALL_APPROACH_STATE_SEARCH_RIGHT: {
        if (new_frame && ball->candidate && ball->color == BALL_COLOR_RED) {
            if (is_red_ball(ball)) {
                after_red_confirmed(controller, &decision, goal, now_us);
            } else {
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ACQUIRE_STOP,
                            BALL_APPROACH_REASON_CANDIDATE, now_us);
            }
            break;
        }
        const bool left = controller->state ==
            BALL_APPROACH_STATE_SEARCH_LEFT;
        const int sweep_motion_ms = left ?
            controller->config.search_left_ms :
            controller->config.search_right_ms;
        if (elapsed_us >= controller->config.search_pulse_ms * 1000LL) {
            const unsigned accumulated = controller->search_motion_ms +
                (unsigned)controller->config.search_pulse_ms;
            const bool sweep_complete =
                accumulated >= (unsigned)sweep_motion_ms;
            controller->search_motion_ms = sweep_complete ? 0U :
                (uint16_t)accumulated;
            controller->settle_next_state = sweep_complete ?
                (left ? BALL_APPROACH_STATE_SEARCH_RIGHT :
                        BALL_APPROACH_STATE_SEARCH_LEFT) :
                controller->state;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_SEARCH_SETTLE,
                        sweep_complete ? BALL_APPROACH_REASON_SEARCH_REVERSE :
                                         BALL_APPROACH_REASON_STARTED,
                        now_us);
            break;
        }
        decision.command = turn_command(
            left ? -1 : 1, controller->config.search_speed);
        break;
    }

    case BALL_APPROACH_STATE_SEARCH_SETTLE:
        if (new_frame && ball->candidate && ball->color == BALL_COLOR_RED) {
            if (is_red_ball(ball)) {
                after_red_confirmed(controller, &decision, goal, now_us);
            } else {
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ACQUIRE_STOP,
                            BALL_APPROACH_REASON_CANDIDATE, now_us);
            }
        } else if (elapsed_us >=
                   controller->config.search_settle_ms * 1000LL) {
            enter_state(controller, &decision,
                        controller->settle_next_state,
                        BALL_APPROACH_REASON_SEARCH_REVERSE, now_us);
        }
        break;

    case BALL_APPROACH_STATE_ACQUIRE_STOP:
        if (new_frame) {
            if (is_red_ball(ball)) {
                after_red_confirmed(controller, &decision, goal, now_us);
            } else if (!ball->candidate ||
                       ball->color != BALL_COLOR_RED) {
                stop_for_unconfirmed_red(
                    controller, &decision, ball, now_us);
            }
        }
        if (controller->state == BALL_APPROACH_STATE_ACQUIRE_STOP &&
            elapsed_us >= controller->config.acquire_timeout_ms * 1000LL) {
            controller->search_motion_ms = 0;
            controller->settle_next_state = BALL_APPROACH_STATE_SEARCH_LEFT;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_SEARCH_SETTLE,
                        BALL_APPROACH_REASON_ACQUIRE_TIMEOUT, now_us);
        }
        break;

    case BALL_APPROACH_STATE_GOAL_SEARCH_LEFT:
    case BALL_APPROACH_STATE_GOAL_SEARCH_RIGHT: {
        if (new_frame && is_blue_goal(goal)) {
            controller->align_frames = 0;
            controller->search_motion_ms = 0;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_PUSH_ALIGN,
                        BALL_APPROACH_REASON_GOAL_SEEN, now_us);
            break;
        }
        const bool left = controller->state ==
            BALL_APPROACH_STATE_GOAL_SEARCH_LEFT;
        const int sweep_motion_ms = left ?
            controller->config.search_left_ms :
            controller->config.search_right_ms;
        if (elapsed_us >= controller->config.search_pulse_ms * 1000LL) {
            const unsigned accumulated = controller->search_motion_ms +
                (unsigned)controller->config.search_pulse_ms;
            const bool sweep_complete =
                accumulated >= (unsigned)sweep_motion_ms;
            controller->search_motion_ms = sweep_complete ? 0U :
                (uint16_t)accumulated;
            controller->settle_next_state = sweep_complete ?
                (left ? BALL_APPROACH_STATE_GOAL_SEARCH_RIGHT :
                        BALL_APPROACH_STATE_GOAL_SEARCH_LEFT) :
                controller->state;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE,
                        sweep_complete ? BALL_APPROACH_REASON_SEARCH_REVERSE :
                                         BALL_APPROACH_REASON_STARTED,
                        now_us);
            break;
        }
        decision.command = turn_command(
            left ? -1 : 1, controller->config.goal_search_speed);
        break;
    }

    case BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE:
        if (new_frame && is_blue_goal(goal)) {
            controller->align_frames = 0;
            controller->search_motion_ms = 0;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_PUSH_ALIGN,
                        BALL_APPROACH_REASON_GOAL_SEEN, now_us);
        } else if (elapsed_us >=
                   controller->config.search_settle_ms * 1000LL) {
            enter_state(controller, &decision,
                        controller->settle_next_state,
                        BALL_APPROACH_REASON_SEARCH_REVERSE, now_us);
        }
        break;

    case BALL_APPROACH_STATE_ROUTE_ALIGN:
        if (!new_frame) break;
        if (!is_red_ball(ball)) {
            stop_for_unconfirmed_red(controller, &decision, ball, now_us);
            break;
        }
        if (!is_blue_goal(goal)) {
            controller->align_frames = 0;
            controller->route_completed = false;
            enter_state(controller, &decision, BALL_APPROACH_STATE_ALIGN,
                        BALL_APPROACH_REASON_ROUTE_BEST_EFFORT, now_us);
            break;
        }
        if (ball_is_inside_clip(controller, ball, red_error)) {
            controller->capture_frames = 1;
            controller->capture_samples = 1;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_CAPTURE_VERIFY,
                        BALL_APPROACH_REASON_CAPTURE_SEEN, now_us);
            break;
        }
        if (abs(red_blue_error) <=
            controller->config.route_deadband_permille) {
            if (controller->align_frames < UINT8_MAX) {
                ++controller->align_frames;
            }
            if (controller->align_frames >=
                controller->config.route_confirm_frames) {
                controller->align_frames = 0;
                controller->route_completed = true;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ALIGN,
                            BALL_APPROACH_REASON_ROUTE_ALIGNED, now_us);
            }
        } else {
            controller->align_frames = 0;
            /* If the goal appears right of the ball, move left so the nearer
             * ball's bearing catches up with the farther goal's bearing. */
            controller->turn_direction = red_blue_error < 0 ? 1 : -1;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_ROUTE_SHIFT,
                        BALL_APPROACH_REASON_ROUTE_SHIFT, now_us);
            decision.command = route_shift_command(
                controller, controller->turn_direction);
        }
        break;

    case BALL_APPROACH_STATE_ROUTE_SHIFT:
        if (new_frame) {
            if (!is_red_ball(ball)) {
                stop_for_unconfirmed_red(
                    controller, &decision, ball, now_us);
                break;
            }
            if (!is_blue_goal(goal)) {
                controller->align_frames = 0;
                controller->route_completed = false;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ALIGN_SETTLE,
                            BALL_APPROACH_REASON_ROUTE_BEST_EFFORT, now_us);
                break;
            }
            const int sample_direction = red_blue_error < 0 ? 1 :
                                         red_blue_error > 0 ? -1 : 0;
            if (abs(red_blue_error) <=
                    controller->config.route_deadband_permille ||
                sample_direction != controller->turn_direction) {
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ROUTE_SETTLE,
                            BALL_APPROACH_REASON_REALIGN, now_us);
                break;
            }
        }
        if (elapsed_us >= controller->config.route_pulse_ms * 1000LL) {
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_ROUTE_SETTLE,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        } else if (controller->state == BALL_APPROACH_STATE_ROUTE_SHIFT) {
            decision.command = route_shift_command(
                controller, controller->turn_direction);
        }
        break;

    case BALL_APPROACH_STATE_ROUTE_SETTLE:
        if (new_frame && !is_red_ball(ball)) {
            stop_for_unconfirmed_red(controller, &decision, ball, now_us);
        } else if (new_frame && !is_blue_goal(goal)) {
            controller->align_frames = 0;
            controller->route_completed = false;
            enter_state(controller, &decision, BALL_APPROACH_STATE_ALIGN,
                        BALL_APPROACH_REASON_ROUTE_BEST_EFFORT, now_us);
        } else if (elapsed_us >=
                   controller->config.align_settle_ms * 1000LL) {
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_ROUTE_ALIGN,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        }
        break;

    case BALL_APPROACH_STATE_ALIGN:
        if (!new_frame) break;
        if (!is_red_ball(ball)) {
            stop_for_unconfirmed_red(controller, &decision, ball, now_us);
            break;
        }
        if (ball_is_inside_clip(controller, ball, red_error)) {
            controller->capture_frames = 1;
            controller->capture_samples = 1;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_CAPTURE_VERIFY,
                        BALL_APPROACH_REASON_CAPTURE_SEEN, now_us);
            break;
        }
        if (!controller->route_completed && is_blue_goal(goal)) {
            begin_route_alignment(controller, &decision, now_us);
            break;
        }
        if (abs(red_error) <= controller->config.align_deadband_permille) {
            if (controller->align_frames < UINT8_MAX) {
                ++controller->align_frames;
            }
            if (controller->align_frames >=
                controller->config.align_confirm_frames) {
                controller->approach_started_us = now_us;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_APPROACH,
                            BALL_APPROACH_REASON_ALIGNED, now_us);
            }
        } else {
            controller->align_frames = 0;
            controller->turn_direction = red_error > 0 ? 1 : -1;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_ALIGN_PULSE,
                        BALL_APPROACH_REASON_ALIGNMENT_PULSE, now_us);
            decision.command = turn_command(
                controller->turn_direction, controller->config.align_speed);
        }
        break;

    case BALL_APPROACH_STATE_ALIGN_PULSE:
        if (new_frame) {
            if (!is_red_ball(ball)) {
                stop_for_unconfirmed_red(
                    controller, &decision, ball, now_us);
                break;
            }
            if (ball_is_inside_clip(controller, ball, red_error)) {
                controller->capture_frames = 1;
                controller->capture_samples = 1;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_CAPTURE_VERIFY,
                            BALL_APPROACH_REASON_CAPTURE_SEEN, now_us);
                break;
            }
            if (!controller->route_completed && is_blue_goal(goal)) {
                begin_route_alignment_after_settle(
                    controller, &decision, now_us);
                break;
            }
            const int sample_direction = red_error > 0 ? 1 :
                                         red_error < 0 ? -1 : 0;
            if (abs(red_error) <= controller->config.align_deadband_permille ||
                sample_direction != controller->turn_direction) {
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ALIGN_SETTLE,
                            BALL_APPROACH_REASON_REALIGN, now_us);
                break;
            }
        }
        if (elapsed_us >= controller->config.align_pulse_ms * 1000LL) {
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_ALIGN_SETTLE,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        } else if (controller->state == BALL_APPROACH_STATE_ALIGN_PULSE) {
            decision.command = turn_command(
                controller->turn_direction, controller->config.align_speed);
        }
        break;

    case BALL_APPROACH_STATE_ALIGN_SETTLE:
        if (new_frame && !is_red_ball(ball)) {
            stop_for_unconfirmed_red(controller, &decision, ball, now_us);
        } else if (new_frame && !controller->route_completed &&
                   is_blue_goal(goal)) {
            begin_route_alignment(controller, &decision, now_us);
        } else if (elapsed_us >=
                   controller->config.align_settle_ms * 1000LL) {
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_ALIGN,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        }
        break;

    case BALL_APPROACH_STATE_APPROACH:
        if (now_us - controller->approach_started_us >=
            controller->config.maximum_approach_ms * 1000LL) {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_APPROACH_TIMEOUT, now_us);
            break;
        }
        if (new_frame) {
            if (!is_red_ball(ball)) {
                /* Never move blindly around a nearby ball.  Stop now, wait,
                 * then restart acquisition instead of latching permanently. */
                enter_anomaly_wait(controller, &decision,
                               BALL_APPROACH_REASON_LOST, now_us);
                break;
            }
            if (ball_is_inside_clip(controller, ball, red_error)) {
                controller->capture_frames = 1;
                controller->capture_samples = 1;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_CAPTURE_VERIFY,
                            BALL_APPROACH_REASON_CAPTURE_SEEN, now_us);
                break;
            }
            if (!controller->route_completed && is_blue_goal(goal)) {
                begin_route_alignment(controller, &decision, now_us);
                break;
            }
            if (abs(red_error) >
                controller->config.realign_threshold_permille) {
                controller->align_frames = 0;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ALIGN_SETTLE,
                            BALL_APPROACH_REASON_REALIGN, now_us);
                break;
            }
            controller->held_command = approach_command(
                controller, ball, red_error, now_us);
        }
        decision.command = controller->held_command;
        break;

    case BALL_APPROACH_STATE_CAPTURE_VERIFY:
        if (!new_frame) break;
        if (controller->capture_samples < UINT8_MAX) {
            ++controller->capture_samples;
        }
        if (ball_is_inside_clip(controller, ball, red_error)) {
            if (controller->capture_frames < UINT8_MAX) {
                ++controller->capture_frames;
            }
            if (controller->capture_frames >=
                controller->config.capture_confirm_frames) {
                after_capture_confirmed(
                    controller, &decision, goal, now_us);
            }
        } else if (controller->capture_samples >=
                   controller->config.capture_verify_max_frames) {
            if (is_red_ball(ball)) {
                /* A visible ball that did not accumulate enough contact
                 * samples is still recoverable.  Return to the stopped ALIGN
                 * decision instead of searching or failing on one jittery
                 * component measurement. */
                controller->capture_frames = 0;
                controller->capture_samples = 0;
                controller->align_frames = 0;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_ALIGN,
                            BALL_APPROACH_REASON_REALIGN, now_us);
            } else {
                enter_anomaly_wait(controller, &decision,
                               BALL_APPROACH_REASON_LOST, now_us);
            }
        }
        break;

    case BALL_APPROACH_STATE_PUSH_ALIGN:
        if (!new_frame) break;
        if (!ball->candidate || ball->color != BALL_COLOR_RED) {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_LOST, now_us);
            break;
        }
        if (ball_is_in_goal(controller, ball, goal)) {
            controller->goal_frames = 1;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_GOAL_VERIFY,
                        BALL_APPROACH_REASON_GOAL_SEEN, now_us);
            break;
        }
        if (!is_blue_goal(goal)) {
            begin_goal_search(controller, &decision, false, now_us);
            break;
        }
        if (abs(blue_error) <= controller->config.align_deadband_permille) {
            if (controller->align_frames < UINT8_MAX) {
                ++controller->align_frames;
            }
            if (controller->align_frames >=
                controller->config.align_confirm_frames) {
                controller->push_motion_started_us = now_us;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_PUSH,
                            BALL_APPROACH_REASON_PUSH_STARTED, now_us);
            }
        } else {
            controller->align_frames = 0;
            /* Translate toward the target instead of pivoting the captured
             * ball.  Moving right shifts a right-side target left in view. */
            controller->turn_direction = blue_error > 0 ? -1 : 1;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_PUSH_ALIGN_PULSE,
                        BALL_APPROACH_REASON_ALIGNMENT_PULSE, now_us);
            decision.command = route_shift_command(
                controller, controller->turn_direction);
        }
        break;

    case BALL_APPROACH_STATE_PUSH_ALIGN_PULSE:
        if (new_frame) {
            if (!ball->candidate || ball->color != BALL_COLOR_RED) {
                enter_anomaly_wait(controller, &decision,
                               BALL_APPROACH_REASON_LOST, now_us);
                break;
            }
            if (ball_is_in_goal(controller, ball, goal)) {
                controller->goal_frames = 1;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_GOAL_VERIFY,
                            BALL_APPROACH_REASON_GOAL_SEEN, now_us);
                break;
            }
            if (!is_blue_goal(goal)) {
                begin_goal_search(controller, &decision, true, now_us);
                break;
            }
            const int sample_direction = blue_error > 0 ? -1 :
                                         blue_error < 0 ? 1 : 0;
            if (abs(blue_error) <= controller->config.align_deadband_permille ||
                sample_direction != controller->turn_direction) {
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE,
                            BALL_APPROACH_REASON_REALIGN, now_us);
                break;
            }
        }
        if (elapsed_us >= controller->config.route_pulse_ms * 1000LL) {
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        } else if (controller->state ==
                   BALL_APPROACH_STATE_PUSH_ALIGN_PULSE) {
            decision.command = route_shift_command(
                controller, controller->turn_direction);
        }
        break;

    case BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE:
        if (new_frame &&
            (!ball->candidate || ball->color != BALL_COLOR_RED)) {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_LOST, now_us);
        } else if (new_frame && !is_blue_goal(goal)) {
            begin_goal_search(controller, &decision, false, now_us);
        } else if (elapsed_us >=
                   controller->config.align_settle_ms * 1000LL) {
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_PUSH_ALIGN,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        }
        break;

    case BALL_APPROACH_STATE_PUSH:
        if (new_frame) {
            if (!ball->candidate || ball->color != BALL_COLOR_RED) {
                enter_anomaly_wait(controller, &decision,
                               BALL_APPROACH_REASON_LOST, now_us);
                break;
            }
            if (ball_is_in_goal(controller, ball, goal)) {
                controller->goal_frames = 1;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_GOAL_VERIFY,
                            BALL_APPROACH_REASON_GOAL_SEEN, now_us);
                break;
            }
            if (!is_blue_goal(goal)) {
                begin_goal_search(controller, &decision, true, now_us);
                break;
            }
            if (abs(blue_error) >
                controller->config.push_realign_threshold_permille) {
                controller->align_frames = 0;
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE,
                            BALL_APPROACH_REASON_REALIGN, now_us);
                break;
            }
            controller->held_command = push_command(
                controller, blue_error, now_us);
        }
        decision.command = controller->held_command;
        break;

    case BALL_APPROACH_STATE_GOAL_VERIFY:
        if (!new_frame) break;
        if (ball_is_in_goal(controller, ball, goal)) {
            if (controller->goal_frames < UINT8_MAX) {
                ++controller->goal_frames;
            }
            if (controller->goal_frames >=
                controller->config.goal_overlap_confirm_frames) {
                enter_state(controller, &decision,
                            BALL_APPROACH_STATE_DONE,
                            BALL_APPROACH_REASON_GOAL_REACHED, now_us);
            }
        } else if (ball->candidate && ball->color == BALL_COLOR_RED) {
            controller->goal_frames = 0;
            controller->align_frames = 0;
            enter_state(controller, &decision,
                        BALL_APPROACH_STATE_PUSH_ALIGN,
                        BALL_APPROACH_REASON_REALIGN, now_us);
        } else {
            enter_anomaly_wait(controller, &decision,
                           BALL_APPROACH_REASON_LOST, now_us);
        }
        break;

    case BALL_APPROACH_STATE_RECOVERY_WAIT:
        /* All anomaly paths are stationary.  Once the requested pause has
         * elapsed and the camera is live again, restart the bounded search
         * from a clean controller state. */
        if (camera->fresh &&
            elapsed_us >= controller->config.recovery_wait_ms * 1000LL) {
            ball_approach_start(controller, now_us);
            decision.transitioned = true;
            decision.reason = BALL_APPROACH_REASON_STARTED;
        }
        break;

    case BALL_APPROACH_STATE_DONE:
    case BALL_APPROACH_STATE_FAILSAFE:
    case BALL_APPROACH_STATE_IDLE:
        break;
    }

    decision.state = controller->state;
    decision.done = controller->state == BALL_APPROACH_STATE_DONE;
    decision.failsafe = controller->state == BALL_APPROACH_STATE_FAILSAFE;
    if (!is_motion_state(controller->state)) {
        decision.command = motor_command_zero();
    }
    return decision;
}

const char *ball_approach_state_name(ball_approach_state_t state)
{
    switch (state) {
    case BALL_APPROACH_STATE_IDLE: return "BALL_IDLE";
    case BALL_APPROACH_STATE_SEARCH_LEFT: return "BALL_SEARCH_LEFT";
    case BALL_APPROACH_STATE_SEARCH_RIGHT: return "BALL_SEARCH_RIGHT";
    case BALL_APPROACH_STATE_SEARCH_SETTLE: return "BALL_SEARCH_SETTLE";
    case BALL_APPROACH_STATE_ACQUIRE_STOP: return "BALL_ACQUIRE_STOP";
    case BALL_APPROACH_STATE_GOAL_SEARCH_LEFT:
        return "BALL_GOAL_SEARCH_LEFT";
    case BALL_APPROACH_STATE_GOAL_SEARCH_RIGHT:
        return "BALL_GOAL_SEARCH_RIGHT";
    case BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE:
        return "BALL_GOAL_SEARCH_SETTLE";
    case BALL_APPROACH_STATE_ROUTE_ALIGN: return "BALL_ROUTE_ALIGN";
    case BALL_APPROACH_STATE_ROUTE_SHIFT: return "BALL_ROUTE_SHIFT";
    case BALL_APPROACH_STATE_ROUTE_SETTLE: return "BALL_ROUTE_SETTLE";
    case BALL_APPROACH_STATE_ALIGN: return "BALL_ALIGN";
    case BALL_APPROACH_STATE_ALIGN_PULSE: return "BALL_ALIGN_PULSE";
    case BALL_APPROACH_STATE_ALIGN_SETTLE: return "BALL_ALIGN_SETTLE";
    case BALL_APPROACH_STATE_APPROACH: return "BALL_APPROACH";
    case BALL_APPROACH_STATE_CAPTURE_VERIFY: return "BALL_CAPTURE_VERIFY";
    case BALL_APPROACH_STATE_PUSH_ALIGN: return "BALL_PUSH_ALIGN";
    case BALL_APPROACH_STATE_PUSH_ALIGN_PULSE:
        return "BALL_PUSH_ALIGN_PULSE";
    case BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE:
        return "BALL_PUSH_ALIGN_SETTLE";
    case BALL_APPROACH_STATE_PUSH: return "BALL_PUSH";
    case BALL_APPROACH_STATE_GOAL_VERIFY: return "BALL_GOAL_VERIFY";
    case BALL_APPROACH_STATE_DONE: return "BALL_DONE";
    case BALL_APPROACH_STATE_FAILSAFE: return "BALL_FAILSAFE";
    case BALL_APPROACH_STATE_RECOVERY_WAIT: return "BALL_RECOVERY_WAIT";
    default: return "BALL_UNKNOWN";
    }
}
