#pragma once

#include <stdbool.h>

typedef struct {
    int lateral_side_permille;
    int lateral_yaw_compensation_permille;
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
    int minimum_active;
    int kp;
    int max_correction;
    int direction_confirm_count;
    int direction_hold_error;
    int boost_command;
    int boost_ms;
    int boost_rearm_ms;
} line_follow_config_t;

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
    int edge_clear_confirm_count;
    int line_confirm_count;
    int uncertain_limit;
    int brake_ms;
    int lateral_speed;
    int lateral_start_speed;
    int motion_boost_ms;
    int lateral_edge_min_ms;
    int lateral_edge_timeout_ms;
    int lateral_clearance_ms;
    int forward_speed;
    int forward_start_speed;
    int forward_bypass_ms;
    int line_search_timeout_ms;
} obstacle_config_t;

typedef struct {
    int press_debounce_ms;
    int release_rearm_ms;
    int startup_guard_ms;
} start_button_config_t;
