#pragma once

#include <stdbool.h>

#include "config_types.h"

typedef struct {
    kiwi_kinematics_config_t kinematics;
    startup_maneuver_config_t startup_maneuver;
    line_follow_config_t line;
    camera_line_config_t camera_line;
    camera_ball_config_t camera_ball;
    ball_approach_config_t ball_approach;
    ball_mission_config_t ball_mission;
    ultrasonic_config_t ultrasonic;
    obstacle_config_t obstacle;
    start_button_config_t button;
    int control_period_ms;
    int telemetry_period_ms;
    int line_monitor_period_ms;
    int default_speed;
    int post_autonomy_ball_delay_ms;
} app_config_t;

extern const app_config_t APP_CONFIG;

bool app_config_validate(const app_config_t *config);
