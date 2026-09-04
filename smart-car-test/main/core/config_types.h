#pragma once

#include <stdbool.h>

typedef struct {
    int lateral_side_permille;
    /* Applied when motion.left is positive. */
    int lateral_yaw_compensation_permille;
    /* Independently applied when motion.left is negative (right strafe). */
    int right_lateral_yaw_compensation_permille;
    int lateral_side_wheel_minimum;
    int motor_b_positive_minimum;
} kiwi_kinematics_config_t;

typedef struct {
    int forward_speed;
    int forward_start_speed;
    int forward_boost_ms;
    int forward_ms;
    int settle_ms;
    int right_turn_speed;
    int right_turn_ms;
} startup_maneuver_config_t;

typedef struct {
    int straight_speed;
    int curve_speed;
    int curve_max;
    int edge_speed;
    int edge_max;
    int search_speed;
    int kp;
    int max_correction;
    int direction_confirm_count;
    int direction_hold_error;
    int single_sensor_inner_command;
    int drive_assist_threshold;
    int drive_assist_command;
    int drive_assist_ms;
    int turn_memory_threshold_permille;
    int turn_memory_release_permille;
    int turn_memory_ms;
    int lost_motion_memory_ms;
    int lost_search_delay_ms;
    int search_direction_threshold_permille;
    int search_primary_ms;
    int search_reverse_ms;
    int search_max_sweep_ms;
    int search_reacquire_frames;
} line_follow_config_t;

typedef struct {
    int roi_left_permille;
    int roi_right_permille;
    int roi_top_permille;
    int roi_bottom_permille;
    int horizontal_scale_permille;
    int center_offset_permille;
    int minimum_contrast;
    int maximum_black_gray;
    int heading_gain_permille;
    int hairpin_near_threshold_permille;
    int hairpin_heading_threshold_permille;
    int hairpin_heading_gain_permille;
    int finish_width_permille;
    int finish_black_permille;
    int history_arm_frames;
    int finish_confirm_frames;
    int fresh_ms;
} camera_line_config_t;

typedef struct {
    /* Weak red admits dim/desaturated edges.  A connected component must also
     * contain enough pixels from the stricter strong-red gate below. */
    int red_minimum;
    int red_dominance;
    int red_ratio_permille;
    int strong_red_dominance;
    int strong_red_ratio_permille;
    /* The live green ball is dark teal: green stays above red while blue may
     * exceed green slightly. Weak and strong gates are calibrated separately. */
    int green_minimum;
    int green_red_dominance;
    int green_blue_tolerance;
    int strong_green_minimum;
    int strong_green_red_dominance;
    int strong_green_blue_tolerance;
    /* Blue uses the same seeded-component pipeline, with thresholds calibrated
     * independently from the live 80x60 image. */
    int blue_minimum;
    int blue_dominance;
    int blue_ratio_permille;
    int strong_blue_dominance;
    int strong_blue_ratio_permille;
    int minimum_strong_pixels;
    int minimum_strong_ratio_permille;
    int minimum_mean_red_dominance;
    int minimum_mean_green_red_dominance;
    int maximum_mean_green_blue_excess;
    int minimum_mean_blue_dominance;
    int minimum_area_permille;
    /* A very small, high-confidence component is accepted only as a far
     * candidate and must remain stable for more decoded frames. */
    int far_minimum_area_permille;
    int far_minimum_confidence_permille;
    int minimum_fill_permille;
    int minimum_roundness_permille;
    int minimum_blue_roundness_permille;
    /* The initial green ball is only about 4x4 pixels at the search distance,
     * so admit a tiny but seeded, round component with slower confirmation. */
    int green_far_minimum_pixels;
    int green_far_minimum_strong_pixels;
    int green_far_minimum_confidence_permille;
    int green_far_minimum_roundness_permille;
    int green_confirm_frames;
    /* Blue destination patches may be only a few decoded pixels at long
     * range.  Keep their raw-pixel gate separate from the red-ball area gate
     * and require a strong, high-confidence connected component. */
    int blue_target_minimum_pixels;
    int blue_target_minimum_strong_pixels;
    int blue_target_minimum_mean_dominance;
    int blue_target_minimum_confidence_permille;
    int blue_target_minimum_roundness_permille;
    /* Blue field patches use their own temporal gate so distant-goal
     * recognition can be relaxed without weakening far red-ball filtering. */
    int blue_target_confirm_frames;
    /* Bright low-chroma pixels are admitted only when locally surrounded by
     * same-color support.  This repairs specular holes without making white
     * objects independent ball candidates. */
    int highlight_minimum;
    int highlight_max_chroma;
    int highlight_target_tolerance;
    int highlight_expand_passes;
    int edge_margin_pixels;
    int tracking_tolerance_permille;
    int confirm_frames;
    int far_confirm_frames;
} camera_ball_config_t;

typedef struct {
    int target_center_x_permille;
    /* Before collecting the ball, strafe until the red ball and blue goal
     * share approximately the same viewing ray. */
    int route_deadband_permille;
    int route_lateral_speed;
    int route_pulse_ms;
    int route_confirm_frames;
    int route_align_timeout_ms;
    /* A locked blue goal may cross the image center, but it cannot jump to a
     * distant second patch in one decoded frame. */
    int goal_tracking_tolerance_permille;
    int align_deadband_permille;
    int realign_threshold_permille;
    int steering_gain_permille;
    int maximum_correction;
    int search_speed;
    int search_pulse_ms;
    int search_left_ms;
    int search_right_ms;
    int search_settle_ms;
    int goal_search_speed;
    int acquire_timeout_ms;
    int align_speed;
    int align_pulse_ms;
    int align_settle_ms;
    int align_confirm_frames;
    int far_forward_speed;
    int medium_forward_speed;
    int near_forward_speed;
    int forward_boost_speed;
    /* During the initial loaded launch pulse, preserve steering while making
     * sure even the weaker A/C wheel clears its static-friction threshold. */
    int forward_boost_min_wheel_speed;
    int forward_boost_ms;
    int medium_y_permille;
    int near_y_permille;
    int capture_center_y_permille;
    int capture_height_permille;
    int capture_area_permille;
    int capture_box_bottom_permille;
    int capture_confirm_frames;
    int capture_verify_max_frames;
    /* Once the ball is in the clip, steer on the blue goal while pushing. */
    int push_speed;
    int push_boost_speed;
    int push_boost_ms;
    int push_steering_gain_permille;
    int push_maximum_correction;
    int push_realign_threshold_permille;
    /* Final blue-goal alignment has a wider, faster confirmation than the
     * earlier ball-centering phase so visual jitter cannot postpone a kick. */
    int push_align_deadband_permille;
    int push_align_confirm_frames;
    /* The passive front guide does not retain a ball.  After confirmed
     * contact, accept a ball rolling away only after sufficient forward image
     * displacement and several stopped frames without a confirmed ball. */
    int delivery_rollaway_minimum_permille;
    int delivery_occlusion_confirm_frames;
    int push_timeout_ms;
    int goal_overlap_margin_permille;
    int goal_overlap_confirm_frames;
    int maximum_total_ms;
    int recovery_wait_ms;
} ball_approach_config_t;

typedef struct {
    /* Hold zero output after red delivery, back away from the goal, execute
     * one fixed clockwise turn, then begin green-ball acquisition. */
    int transition_settle_ms;
    int red_exit_reverse_speed;
    int red_exit_reverse_boost_speed;
    int red_exit_reverse_boost_ms;
    int red_exit_reverse_ms;
    int red_exit_reverse_settle_ms;
    int green_entry_right_turn_speed;
    int green_entry_right_turn_ms;
    int green_entry_turn_settle_ms;
} ball_mission_config_t;

typedef struct {
    int timeout_us;
    int period_ms;
    int fresh_ms;
    int jump_mm;
    int cluster_mm;
    int outlier_confirm_count;
    int lost_confirm_count;
} ultrasonic_config_t;

typedef struct {
    bool bypass_enabled;
    int stop_mm;
    int clear_confirm_count;
    int no_echo_limit;
    int uncertain_limit;
    int brake_ms;
    int lateral_speed;
    int lateral_start_speed;
    int motion_boost_ms;
    int left_strafe_ms;
    int left_heading_trim_speed;
    int left_heading_trim_ms;
    int forward_speed;
    int forward_start_speed;
    int forward_drive_ms;
    int right_lateral_start_speed;
    /* Signed body yaw command; negative counteracts clockwise launch drift. */
    int right_lateral_start_clockwise;
    int right_lateral_ramp_ms;
    int right_strafe_ms;
    int post_bypass_forward_ms;
} obstacle_config_t;

typedef struct {
    int press_debounce_ms;
    int release_rearm_ms;
    int startup_guard_ms;
} start_button_config_t;
