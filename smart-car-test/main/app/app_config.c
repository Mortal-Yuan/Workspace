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
        /* Slight residual counter-clockwise drift remained at 50%. */
        .lateral_yaw_compensation_permille = 550,
        /* Right strafe still rotated clockwise severely at 30%. */
        .right_lateral_yaw_compensation_permille = 500,
        /* A/C stalled at 226 during the 20% compensation trial. */
        .lateral_side_wheel_minimum = 300,
        /* B positive stalls at 340; keep that direction above its dead zone. */
        .motor_b_positive_minimum = 460,
    },
    .line = {
        /* Keep the 8/25 geometry while adding selective drive-wheel authority. */
        /* Normal line-follow motion raised 20%; search is unchanged. */
        .straight_speed = 348,
        .curve_speed = 276,
        .curve_max = 384,
        .edge_speed = 233,
        .edge_max = 336,
        /* 192 raised 10% and rounded to the nearest integer command. */
        .search_speed = 211,
        .kp = 120,
        .max_correction = 300,
        .direction_confirm_count = 3,
        .direction_hold_error = 4,
        .single_sensor_inner_command = 100,
        /* Never boost the low-demand inside wheel. */
        .drive_assist_threshold = 200,
        .drive_assist_command = 500,
        .drive_assist_ms = 150,
        /* Delay only the onset of a newly established camera turn. */
        .turn_memory_threshold_permille = 180,
        .turn_memory_release_permille = 80,
        .turn_memory_ms = 180,
        /* Continue the last proven trajectory across a brief near blind spot. */
        .lost_motion_memory_ms = 120,
        /* Expanding in-place search: 1.2, 2.4, ... 7.2 second legs. */
        .lost_search_delay_ms = 150,
        .search_direction_threshold_permille = 100,
        .search_primary_ms = 1200,
        .search_reverse_ms = 2400,
        .search_max_sweep_ms = 7200,
        .search_reacquire_frames = 3,
    },
    .camera_line = {
        /* Analyze the nearest central track window in the native view. */
        .roi_left_permille = 250,
        .roi_right_permille = 750,
        .roi_top_permille = 600,
        .roi_bottom_permille = 930,
        .horizontal_scale_permille = 1000,
        /* 2026-08-31 geometric-center recalibration: +202 -> 0. */
        .center_offset_permille = -165,
        .minimum_contrast = 30,
        /* Shadows may be relatively dark but are not black tape.  Apply this
         * absolute ceiling after the adaptive Otsu calculation. */
        .maximum_black_gray = 120,
        .heading_gain_permille = 500,
        /* Avoid near/far cancellation when a connected line crosses the view
         * in a sharp hairpin. */
        .hairpin_near_threshold_permille = 160,
        .hairpin_heading_threshold_permille = 300,
        .hairpin_heading_gain_permille = 100,
        /* T finish: demand a near-full-width crossbar, but permit a thinner
         * strip than the retired broad-black-patch area gate. */
        .finish_width_permille = 800,
        .finish_black_permille = 200,
        .history_arm_frames = 3,
        .finish_confirm_frames = 2,
        /* Three to four missing 15 fps frames stop autonomous motion. */
        .fresh_ms = 350,
    },
    .camera_ball = {
        /* Initial red-ball thresholds; field telemetry exposes mean RGB. */
        .red_minimum = 45,
        .red_dominance = 8,
        .red_ratio_permille = 380,
        /* Reject the weak red background while retaining dim ball edges. */
        .minimum_mean_red_dominance = 40,
        /* At 80x60 this starts at roughly 24 connected red pixels. */
        .minimum_area_permille = 5,
        .minimum_fill_permille = 350,
        .minimum_roundness_permille = 600,
        .edge_margin_pixels = 2,
        .tracking_tolerance_permille = 200,
        .confirm_frames = 3,
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
        /* Field retune: approach closer before starting the bypass. */
        .stop_mm = 60,
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
        /* The 1231 ms field result still travelled slightly too far. */
        .left_strafe_ms = 1170,
        .forward_speed = 400,
        .forward_start_speed = 500,
        /* Keep the longer segment after restoring the 60 mm trigger so the
         * car clears roughly 40 mm farther beyond the obstacle. */
        .forward_drive_ms = 1191,
        /* Increase the last 800 ms right segment by 20%. */
        .right_strafe_ms = 960,
        /* After the right strafe has reacquired and confirmed the line, use
         * normal camera steering for another second, then finish. */
        .post_bypass_forward_ms = 1000,
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
        config->kinematics.lateral_yaw_compensation_permille >= -1000 &&
        config->kinematics.lateral_yaw_compensation_permille <= 1000 &&
        config->kinematics.right_lateral_yaw_compensation_permille >= -1000 &&
        config->kinematics.right_lateral_yaw_compensation_permille <= 1000 &&
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
        config->obstacle.post_bypass_forward_ms > 0 &&
        config->obstacle.post_bypass_forward_ms <= 5000 &&
        valid_command(config->line.straight_speed) &&
        valid_command(config->line.curve_speed) &&
        valid_command(config->line.curve_max) &&
        valid_command(config->line.edge_speed) &&
        valid_command(config->line.edge_max) &&
        valid_command(config->line.search_speed) &&
        config->line.search_speed > 0 &&
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
        config->line.turn_memory_threshold_permille > 0 &&
        config->line.turn_memory_threshold_permille <= 1000 &&
        config->line.turn_memory_release_permille >= 0 &&
        config->line.turn_memory_release_permille <
            config->line.turn_memory_threshold_permille &&
        config->line.turn_memory_ms >= 0 &&
        config->line.turn_memory_ms <= 1000 &&
        config->line.lost_motion_memory_ms >= 0 &&
        config->line.lost_motion_memory_ms <=
            config->line.lost_search_delay_ms &&
        config->line.lost_search_delay_ms >= 0 &&
        config->line.lost_search_delay_ms <= 1000 &&
        config->line.search_direction_threshold_permille > 0 &&
        config->line.search_direction_threshold_permille <= 1000 &&
        config->line.search_primary_ms > 0 &&
        config->line.search_primary_ms <= 10000 &&
        config->line.search_reverse_ms > 0 &&
        config->line.search_reverse_ms <= 20000 &&
        config->line.search_reverse_ms >
            config->line.search_primary_ms &&
        config->line.search_max_sweep_ms >=
            config->line.search_reverse_ms &&
        config->line.search_max_sweep_ms <= 60000 &&
        config->line.search_reacquire_frames >= 2 &&
        config->line.search_reacquire_frames <= 20 &&
        config->camera_line.roi_left_permille >= 0 &&
        config->camera_line.roi_left_permille <
            config->camera_line.roi_right_permille &&
        config->camera_line.roi_right_permille <= 1000 &&
        config->camera_line.roi_top_permille >= 0 &&
        config->camera_line.roi_top_permille <
            config->camera_line.roi_bottom_permille &&
        config->camera_line.roi_bottom_permille <= 1000 &&
        config->camera_line.horizontal_scale_permille >= 1000 &&
        config->camera_line.horizontal_scale_permille <= 4000 &&
        config->camera_line.center_offset_permille >= -500 &&
        config->camera_line.center_offset_permille <= 500 &&
        config->camera_line.minimum_contrast >= 10 &&
        config->camera_line.minimum_contrast <= 255 &&
        config->camera_line.maximum_black_gray > 0 &&
        config->camera_line.maximum_black_gray <= 255 &&
        config->camera_line.heading_gain_permille >= 0 &&
        config->camera_line.heading_gain_permille <= 2000 &&
        config->camera_line.hairpin_near_threshold_permille >= 0 &&
        config->camera_line.hairpin_near_threshold_permille <= 1000 &&
        config->camera_line.hairpin_heading_threshold_permille >= 0 &&
        config->camera_line.hairpin_heading_threshold_permille <= 2000 &&
        config->camera_line.hairpin_heading_gain_permille >= 0 &&
        config->camera_line.hairpin_heading_gain_permille <=
            config->camera_line.heading_gain_permille &&
        config->camera_line.finish_width_permille > 0 &&
        config->camera_line.finish_width_permille <= 1000 &&
        config->camera_line.finish_black_permille > 0 &&
        config->camera_line.finish_black_permille <= 1000 &&
        config->camera_line.history_arm_frames > 0 &&
        config->camera_line.history_arm_frames <= 20 &&
        config->camera_line.finish_confirm_frames > 0 &&
        config->camera_line.finish_confirm_frames <= 20 &&
        config->camera_line.fresh_ms >= 100 &&
        config->camera_ball.red_minimum >= 0 &&
        config->camera_ball.red_minimum <= 255 &&
        config->camera_ball.red_dominance > 0 &&
        config->camera_ball.red_dominance <= 255 &&
        config->camera_ball.red_ratio_permille > 333 &&
        config->camera_ball.red_ratio_permille <= 1000 &&
        config->camera_ball.minimum_mean_red_dominance > 0 &&
        config->camera_ball.minimum_mean_red_dominance <= 255 &&
        config->camera_ball.minimum_area_permille > 0 &&
        config->camera_ball.minimum_area_permille <= 500 &&
        config->camera_ball.minimum_fill_permille > 0 &&
        config->camera_ball.minimum_fill_permille <= 1000 &&
        config->camera_ball.minimum_roundness_permille > 0 &&
        config->camera_ball.minimum_roundness_permille <= 1000 &&
        config->camera_ball.edge_margin_pixels >= 0 &&
        config->camera_ball.edge_margin_pixels <= 10 &&
        config->camera_ball.tracking_tolerance_permille > 0 &&
        config->camera_ball.tracking_tolerance_permille <= 1000 &&
        config->camera_ball.confirm_frames > 0 &&
        config->camera_ball.confirm_frames <= 20 &&
        config->button.press_debounce_ms > 0 &&
        config->button.release_rearm_ms > 0 &&
        config->button.startup_guard_ms >= 0 &&
        config->default_speed >= 100 && config->default_speed <= 800;
}
