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
        /* Left compensation raised to match the independently tuned right. */
        .lateral_yaw_compensation_permille = 500,
        /* Right strafe still rotated clockwise severely at 30%. */
        .right_lateral_yaw_compensation_permille = 500,
        /* A/C stalled at 226 during the 20% compensation trial. */
        .lateral_side_wheel_minimum = 300,
        /* B positive stalls at 340; keep that direction above its dead zone. */
        .motor_b_positive_minimum = 460,
    },
    .line = {
        /* Keep the 8/25 geometry while adding selective drive-wheel authority. */
        .straight_speed = 360,
        .curve_speed = 250,
        .curve_max = 400,
        .edge_speed = 190,
        .edge_max = 320,
        .search_speed = 320,
        .kp = 100,
        .max_correction = 250,
        .direction_confirm_count = 3,
        .direction_hold_error = 4,
        .single_sensor_inner_command = 100,
        /* Never boost the low-demand inside wheel. */
        .drive_assist_threshold = 200,
        .drive_assist_command = 500,
        .drive_assist_ms = 150,
    },
    .ultrasonic = {
        /* Allow the module's completed long no-return pulse to be observed. */
        .timeout_us = 45000,
        .period_ms = 70,
        .fresh_ms = 250,
        .jump_mm = 150,
        .cluster_mm = 80,
        .outlier_confirm_count = 2,
        .lost_confirm_count = 3,
    },
    .obstacle = {
        /* Enabled for the first ground-level distance calibration. */
        .bypass_enabled = true,
        .stop_mm = 100,
        .clear_confirm_count = 3,
        .line_confirm_count = 5,
        /* Retained for diagnostics; open-space no-Echo does not stop CLEAR. */
        .no_echo_limit = 3,
        .uncertain_limit = 3,
        .brake_ms = 150,
        /* Restore the proven lateral speeds; left/right share these values. */
        .lateral_speed = 380,
        .lateral_start_speed = 500,
        .motion_boost_ms = 150,
        /* Reduce the last 1296 ms left segment by another 5%. */
        .left_strafe_ms = 1231,
        .forward_speed = 400,
        .forward_start_speed = 500,
        /* Stop 40 mm earlier at 100 mm, then recover that distance forward. */
        .forward_drive_ms = 1191,
        /* Increase the last 800 ms right segment by 20%. */
        .right_strafe_ms = 960,
    },
    .button = {
        .press_debounce_ms = 50,
        .release_rearm_ms = 80,
        .startup_guard_ms = 500,
    },
    .control_period_ms = 20,
    .telemetry_period_ms = 500,
    .line_monitor_period_ms = 100,
    .default_speed = 400,
};

bool app_config_validate(const app_config_t *config)
{
    return config != NULL &&
        config->control_period_ms > 0 &&
        config->kinematics.lateral_side_permille > 0 &&
        config->kinematics.lateral_side_permille <= 1000 &&
        config->kinematics.lateral_yaw_compensation_permille >= -500 &&
        config->kinematics.lateral_yaw_compensation_permille <= 500 &&
        config->kinematics.right_lateral_yaw_compensation_permille >= -500 &&
        config->kinematics.right_lateral_yaw_compensation_permille <= 500 &&
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
        config->obstacle.line_confirm_count > 0 &&
        config->obstacle.line_confirm_count <= 255 &&
        config->obstacle.no_echo_limit > 0 &&
        config->obstacle.no_echo_limit <= 255 &&
        config->obstacle.uncertain_limit > 0 &&
        config->obstacle.uncertain_limit <= 255 &&
        config->obstacle.brake_ms > 0 &&
        valid_command(config->obstacle.lateral_speed) &&
        config->obstacle.lateral_speed > 0 &&
        valid_command(config->obstacle.lateral_start_speed) &&
        config->obstacle.lateral_start_speed >=
            config->obstacle.lateral_speed &&
        config->obstacle.motion_boost_ms >= 0 &&
        config->obstacle.left_strafe_ms > 0 &&
        valid_command(config->obstacle.forward_speed) &&
        config->obstacle.forward_speed > 0 &&
        valid_command(config->obstacle.forward_start_speed) &&
        config->obstacle.forward_start_speed >=
            config->obstacle.forward_speed &&
        config->obstacle.forward_drive_ms > 0 &&
        config->obstacle.right_strafe_ms > 0 &&
        valid_command(config->line.straight_speed) &&
        valid_command(config->line.curve_speed) &&
        valid_command(config->line.curve_max) &&
        valid_command(config->line.edge_speed) &&
        valid_command(config->line.edge_max) &&
        valid_command(config->line.search_speed) &&
        config->line.kp > 0 &&
        valid_command(config->line.max_correction) &&
        config->line.direction_confirm_count > 0 &&
        config->line.single_sensor_inner_command > 0 &&
        valid_command(config->line.single_sensor_inner_command) &&
        config->line.single_sensor_inner_command <
            config->line.drive_assist_threshold &&
        config->line.drive_assist_threshold > 0 &&
        valid_command(config->line.drive_assist_threshold) &&
        config->line.drive_assist_command >=
            config->line.drive_assist_threshold &&
        valid_command(config->line.drive_assist_command) &&
        config->line.drive_assist_ms > 0 &&
        config->button.press_debounce_ms > 0 &&
        config->button.release_rearm_ms > 0 &&
        config->button.startup_guard_ms >= 0 &&
        config->default_speed >= 100 && config->default_speed <= 800;
}
