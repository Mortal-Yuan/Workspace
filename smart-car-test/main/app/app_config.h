#pragma once

#include <stdbool.h>

#include "config_types.h"

typedef struct {
    kiwi_kinematics_config_t kinematics;
    line_follow_config_t line;
    ultrasonic_config_t ultrasonic;
    obstacle_config_t obstacle;
    start_button_config_t button;
    int control_period_ms;
    int telemetry_period_ms;
    int line_monitor_period_ms;
    int default_speed;
} app_config_t;

extern const app_config_t APP_CONFIG;

bool app_config_validate(const app_config_t *config);
