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
} line_follow_config_t;

typedef struct {
    int roi_left_permille;
    int roi_right_permille;
    int roi_top_permille;
    int roi_bottom_permille;
    int center_offset_permille;
    int minimum_contrast;
    int minimum_column_fill_permille;
    int minimum_component_pixels;
    int finish_width_permille;
    int finish_black_permille;
    int fresh_ms;
} camera_line_config_t;

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
    int line_confirm_count;
    int no_echo_limit;
    int uncertain_limit;
    int brake_ms;
    int lateral_speed;
    int lateral_start_speed;
    int motion_boost_ms;
    int left_strafe_ms;
    int forward_speed;
    int forward_start_speed;
    int forward_drive_ms;
    int right_strafe_ms;
} obstacle_config_t;

typedef struct {
    int press_debounce_ms;
    int release_rearm_ms;
    int startup_guard_ms;
} start_button_config_t;
