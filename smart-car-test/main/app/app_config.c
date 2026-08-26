#include "app_config.h"

#include <stddef.h>

_Static_assert(1000 <= 1023, "motor command range must fit 10-bit PWM");

static bool valid_command(int value)
{
    return value >= 0 && value <= 1000;
}

const app_config_t APP_CONFIG = {
    .kinematics = {
        .lateral_side_permille = 866,
        /* Suspended calibration: cancel left-on-left/right-on-right yaw. */
        .lateral_yaw_compensation_permille = 200,
        /* A/C stalled at 226 during the 20% compensation trial. */
        .lateral_side_wheel_minimum = 300,
        /* B positive stalls at 340; keep that direction above its dead zone. */
        .motor_b_positive_minimum = 460,
    },
    .line = {
        .straight_speed = 420,
        .curve_speed = 310,
        .curve_max = 500,
        .edge_speed = 280,
        .edge_max = 450,
        .search_speed = 360,
        .minimum_active = 220,
        .kp = 75,
        .max_correction = 220,
        .direction_confirm_count = 3,
        .direction_hold_error = 4,
        .boost_command = 500,
        .boost_ms = 150,
        .boost_rearm_ms = 120,
    },
    .ultrasonic = {
        .timeout_us = 30000,
        .period_ms = 60,
        .fresh_ms = 250,
        .jump_mm = 150,
        .cluster_mm = 80,
        .outlier_confirm_count = 2,
        .lost_confirm_count = 3,
    },
    .obstacle = {
        /* Enabled for the first bounded field calibration. */
        .bypass_enabled = true,
        .stop_mm = 200,
        .clear_confirm_count = 3,
        .edge_clear_confirm_count = 3,
        .line_confirm_count = 5,
        .uncertain_limit = 3,
        .brake_ms = 150,
        .lateral_speed = 380,
        .lateral_start_speed = 500,
        .motion_boost_ms = 150,
        .lateral_edge_min_ms = 200,
        .lateral_edge_timeout_ms = 5000,
        /* Placeholders only; these do not represent calibrated distances. */
        .lateral_clearance_ms = 800,
        .forward_speed = 400,
        .forward_start_speed = 500,
        .forward_bypass_ms = 2500,
        .line_search_timeout_ms = 5000,
    },
    .button = {
        .press_debounce_ms = 50,
        .release_rearm_ms = 80,
        .startup_guard_ms = 500,
    },
    .control_period_ms = 20,
    .telemetry_period_ms = 500,
    .line_monitor_period_ms = 100,
    .default_speed = 450,
};

bool app_config_validate(const app_config_t *config)
{
    return config != NULL &&
        config->control_period_ms > 0 &&
        config->kinematics.lateral_side_permille > 0 &&
        config->kinematics.lateral_side_permille <= 1000 &&
        config->kinematics.lateral_yaw_compensation_permille >= -500 &&
        config->kinematics.lateral_yaw_compensation_permille <= 500 &&
        valid_command(config->kinematics.lateral_side_wheel_minimum) &&
        valid_command(config->kinematics.motor_b_positive_minimum) &&
        config->telemetry_period_ms > 0 &&
        config->line_monitor_period_ms > 0 &&
        config->ultrasonic.period_ms > config->control_period_ms &&
        config->ultrasonic.timeout_us > 0 &&
        config->ultrasonic.timeout_us <
            config->ultrasonic.period_ms * 1000LL &&
        config->ultrasonic.fresh_ms >= config->ultrasonic.period_ms &&
        config->ultrasonic.jump_mm > 0 &&
        config->ultrasonic.cluster_mm > 0 &&
        config->ultrasonic.outlier_confirm_count > 0 &&
        config->ultrasonic.lost_confirm_count > 0 &&
        config->obstacle.stop_mm >= 20 && config->obstacle.stop_mm <= 4000 &&
        config->obstacle.clear_confirm_count > 0 &&
        config->obstacle.clear_confirm_count <= 255 &&
        config->obstacle.edge_clear_confirm_count > 0 &&
        config->obstacle.edge_clear_confirm_count <= 255 &&
        config->obstacle.line_confirm_count > 0 &&
        config->obstacle.line_confirm_count <= 255 &&
        config->obstacle.uncertain_limit > 0 &&
        config->obstacle.uncertain_limit <= 255 &&
        config->obstacle.brake_ms > 0 &&
        valid_command(config->obstacle.lateral_speed) &&
        config->obstacle.lateral_speed > 0 &&
        valid_command(config->obstacle.lateral_start_speed) &&
        config->obstacle.lateral_start_speed >=
            config->obstacle.lateral_speed &&
        config->obstacle.motion_boost_ms >= 0 &&
        config->obstacle.lateral_edge_min_ms >= 0 &&
        config->obstacle.lateral_edge_timeout_ms >
            config->obstacle.lateral_edge_min_ms &&
        config->obstacle.lateral_clearance_ms > 0 &&
        valid_command(config->obstacle.forward_speed) &&
        config->obstacle.forward_speed > 0 &&
        valid_command(config->obstacle.forward_start_speed) &&
        config->obstacle.forward_start_speed >=
            config->obstacle.forward_speed &&
        config->obstacle.forward_bypass_ms > 0 &&
        config->obstacle.line_search_timeout_ms > 0 &&
        valid_command(config->line.straight_speed) &&
        valid_command(config->line.curve_speed) &&
        valid_command(config->line.curve_max) &&
        valid_command(config->line.edge_speed) &&
        valid_command(config->line.edge_max) &&
        valid_command(config->line.search_speed) &&
        config->line.minimum_active > 0 &&
        valid_command(config->line.minimum_active) &&
        config->line.kp > 0 &&
        valid_command(config->line.max_correction) &&
        config->line.direction_confirm_count > 0 &&
        config->line.boost_ms > 0 && config->line.boost_rearm_ms > 0 &&
        valid_command(config->line.boost_command) &&
        config->button.press_debounce_ms > 0 &&
        config->button.release_rearm_ms > 0 &&
        config->button.startup_guard_ms >= 0 &&
        config->default_speed >= 100 && config->default_speed <= 800;
}
