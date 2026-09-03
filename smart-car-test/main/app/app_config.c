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
    .startup_maneuver = {
        /* Open-loop field calibration: the existing 1074 ms forward segment
         * is approximately 20 cm at 400, including its 150 ms launch boost. */
        .forward_speed = 400,
        .forward_start_speed = 500,
        .forward_boost_ms = 150,
        .forward_ms = 1074,
        /* Stop before reversing the right wheel, and again before vision
         * takes control, to reduce inertial overshoot. */
        .settle_ms = 150,
        /* The verified right-yaw basis is A=+420,B=-420,C=-420.  Reduce the
         * preceding 267 ms estimate to three quarters: 200.25 ms, rounded to 200. */
        .right_turn_speed = 420,
        .right_turn_ms = 200,
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
        /* Weak red keeps dim/desaturated ball edges connected. */
        .red_minimum = 45,
        .red_dominance = 8,
        .red_ratio_permille = 380,
        /* Strong red seeds prevent weak reddish scenery from becoming a ball. */
        .strong_red_dominance = 40,
        .strong_red_ratio_permille = 420,
        /* Live 80x60 center rectangle: about RGB 125/157/251.  The stronger
         * mean-blue gate rejects the paler blue background components. */
        .blue_minimum = 70,
        .blue_dominance = 16,
        .blue_ratio_permille = 390,
        /* The newly placed distant side targets measure only about
         * RGB 110/124/162 and 102/117/144.  A 24-level dominance is enough
         * to seed those tiny components; larger regions still need the
         * stricter mean-blue gate below. */
        .strong_blue_dominance = 24,
        .strong_blue_ratio_permille = 390,
        .minimum_strong_pixels = 4,
        .minimum_strong_ratio_permille = 80,
        .minimum_mean_red_dominance = 24,
        .minimum_mean_blue_dominance = 55,
        /* At 80x60 this starts at roughly 20 repaired support pixels. */
        .minimum_area_permille = 4,
        /* The 40 cm calibration is a stable 4x4 component at 2 permille and
         * about 0.91 confidence.  Keep a separate, slower-confirming gate so
         * ordinary weak red specks do not loosen the near-ball detector. */
        .far_minimum_area_permille = 2,
        .far_minimum_confidence_permille = 900,
        .minimum_fill_permille = 350,
        .minimum_roundness_permille = 600,
        /* The calibrated blue target is roughly 12x5 pixels (417 permille). */
        .minimum_blue_roundness_permille = 350,
        /* At long range either side target can shrink to a three-pixel
         * component.  Two strong-blue seeds plus temporal confirmation keep
         * this permissive spatial gate from accepting isolated sensor noise. */
        .blue_target_minimum_pixels = 3,
        .blue_target_minimum_strong_pixels = 2,
        .blue_target_minimum_mean_dominance = 24,
        .blue_target_minimum_confidence_permille = 720,
        .blue_target_minimum_roundness_permille = 250,
        /* Two local majority-growth passes fill a small white specular hole. */
        .highlight_minimum = 160,
        .highlight_max_chroma = 48,
        .highlight_target_tolerance = 12,
        .highlight_expand_passes = 2,
        .edge_margin_pixels = 2,
        .tracking_tolerance_permille = 200,
        .confirm_frames = 3,
        .far_confirm_frames = 5,
    },
    .ball_approach = {
        /* P0 contact samples center at +23; +25 is the camera/clip axis. */
        .target_center_x_permille = 25,
        /* The kiwi chassis can translate sideways without intentionally
         * changing heading.  Short stopped-frame pulses place the car on the
         * red-ball/blue-goal line before it commits to the approach. */
        .route_deadband_permille = 100,
        .route_lateral_speed = 300,
        .route_pulse_ms = 80,
        .route_confirm_frames = 3,
        .route_align_timeout_ms = 2500,
        .align_deadband_permille = 60,
        .realign_threshold_permille = 180,
        .steering_gain_permille = 500,
        .maximum_correction = 100,
        /* Search always starts left, then covers twice that arc to the right.
         * Turn in short, fast increments with a stopped observation between
         * pulses instead of continuously spinning through the target. */
        .search_speed = 360,
        .search_pulse_ms = 80,
        .search_left_ms = 800,
        .search_right_ms = 1600,
        .search_settle_ms = 200,
        /* Once the ball is captured, retain the gentler target-search speed,
         * but use the same pulse-and-observe cadence as red-ball search. */
        .goal_search_speed = 260,
        .acquire_timeout_ms = 1500,
        /* Short pivot pulses leave a stopped frame between corrections. */
        .align_speed = 300,
        .align_pulse_ms = 80,
        .align_settle_ms = 200,
        .align_confirm_frames = 3,
        /* P40/P30/P20/P10/P0 center-y calibration: 408/454/518/651/837.
         * Retain the original drive levels: the 30%-reduced experiment could
         * not reliably overcome static friction. */
        .far_forward_speed = 300,
        .medium_forward_speed = 250,
        .near_forward_speed = 210,
        .forward_boost_speed = 360,
        .forward_boost_ms = 100,
        .medium_y_permille = 520,
        .near_y_permille = 700,
        /* P10 is only y=651/bottom about 729.  Two live contact trials were
         * y=780..814 and bottom=864..898, so leave more than 100 permille of
         * separation from P10 while accepting the one-row 80x60 variation. */
        .capture_center_y_permille = 760,
        /* Position is the primary contact cue; size remains a permissive
         * secondary guard against a small distant red speck. */
        .capture_height_permille = 130,
        .capture_area_permille = 10,
        /* Accept row 51 of 59 (864 permille); the former row-52 threshold
         * rejected the second live in-clip trial by exactly one pixel. */
        .capture_box_bottom_permille = 850,
        .capture_confirm_frames = 2,
        .capture_verify_max_frames = 5,
        .maximum_approach_ms = 12000,
        /* Increase both launch and steady effort because the loaded clip no
         * longer overcomes static friction at the former 360/250 levels. */
        .push_speed = 300,
        .push_boost_speed = 420,
        .push_boost_ms = 180,
        .push_steering_gain_permille = 500,
        .push_maximum_correction = 80,
        .push_realign_threshold_permille = 180,
        .push_timeout_ms = 10000,
        /* Completion now requires the red center to enter the measured blue
         * box itself; do not accept the former 60-permille exterior margin. */
        .goal_overlap_margin_permille = 0,
        .goal_overlap_confirm_frames = 3,
        .maximum_total_ms = 45000,
        /* Vision loss and policy timeouts stop immediately, wait here, then
         * restart acquisition instead of latching BALL_FAILSAFE. */
        .recovery_wait_ms = 2000,
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
        /* Field retune: start the bypass 2 cm earlier (6 cm -> 8 cm). */
        .stop_mm = 80,
        .clear_confirm_count = 3,
        /* Retained for diagnostics; open-space no-Echo does not stop CLEAR. */
        .no_echo_limit = 3,
        .uncertain_limit = 3,
        .brake_ms = 150,
        /* Restore the proven lateral speeds; left/right share these values. */
        .lateral_speed = 380,
        .lateral_start_speed = 500,
        .motion_boost_ms = 150,
        /* Field request: increase the preceding 936 ms segment by 10%;
         * 936 * 1.10 = 1029.6 ms, rounded to the nearest millisecond. */
        .left_strafe_ms = 1030,
        .forward_speed = 400,
        .forward_start_speed = 500,
        /* Field request: increase the preceding 826 ms segment by 30%;
         * 826 * 1.30 = 1073.8 ms, rounded to the nearest millisecond. */
        .forward_drive_ms = 1074,
        /* Right strafe no longer uses the 500-command launch boost.  Ramp
         * from the lowest effective lateral command to 380 over 400 ms to
         * reduce the tyre torque step that caused slipping. */
        .right_lateral_start_speed = 300,
        /* A/C immediately hit their 300 floors while B otherwise starts at
         * only 460, which caused a short clockwise yaw.  Fade this small
         * counter-clockwise bias out with the lateral ramp so B starts near
         * 510 and still finishes at the proven 570 steady command. */
        .right_lateral_start_clockwise = -60,
        .right_lateral_ramp_ms = 400,
        /* Preserve approximately the preceding 1008 ms profile's command
         * integral after replacing 150 ms at 500 with the slow ramp:
         * old = 500*150 + 380*(1008-150) = 401040 command-ms;
         * new = (300+380)/2*400 + 380*(1097-400) ~= 400860. */
        .right_strafe_ms = 1097,
        /* After the full right strafe, ignore line input and drive straight
         * for 5% longer than the preceding 500 ms. */
        .post_bypass_forward_ms = 525,
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
        valid_command(config->startup_maneuver.forward_speed) &&
        config->startup_maneuver.forward_speed > 0 &&
        valid_command(config->startup_maneuver.forward_start_speed) &&
        config->startup_maneuver.forward_start_speed >=
            config->startup_maneuver.forward_speed &&
        config->startup_maneuver.forward_boost_ms >= 0 &&
        config->startup_maneuver.forward_boost_ms <=
            config->startup_maneuver.forward_ms &&
        config->startup_maneuver.forward_ms > 0 &&
        config->startup_maneuver.forward_ms <= 10000 &&
        config->startup_maneuver.settle_ms >= 0 &&
        config->startup_maneuver.settle_ms <= 2000 &&
        valid_command(config->startup_maneuver.right_turn_speed) &&
        config->startup_maneuver.right_turn_speed > 0 &&
        config->startup_maneuver.right_turn_ms > 0 &&
        config->startup_maneuver.right_turn_ms <= 5000 &&
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
        valid_command(config->obstacle.right_lateral_start_speed) &&
        config->obstacle.right_lateral_start_speed > 0 &&
        config->obstacle.right_lateral_start_speed <=
            config->obstacle.lateral_speed &&
        config->obstacle.right_lateral_start_clockwise >= -1000 &&
        config->obstacle.right_lateral_start_clockwise <= 1000 &&
        config->obstacle.right_lateral_ramp_ms > 0 &&
        config->obstacle.right_strafe_ms > 0 &&
        config->obstacle.right_lateral_ramp_ms <
            config->obstacle.right_strafe_ms &&
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
        config->camera_ball.strong_red_dominance >=
            config->camera_ball.red_dominance &&
        config->camera_ball.strong_red_dominance <= 255 &&
        config->camera_ball.strong_red_ratio_permille >=
            config->camera_ball.red_ratio_permille &&
        config->camera_ball.strong_red_ratio_permille <= 1000 &&
        config->camera_ball.blue_minimum >= 0 &&
        config->camera_ball.blue_minimum <= 255 &&
        config->camera_ball.blue_dominance > 0 &&
        config->camera_ball.blue_dominance <= 255 &&
        config->camera_ball.blue_ratio_permille > 333 &&
        config->camera_ball.blue_ratio_permille <= 1000 &&
        config->camera_ball.strong_blue_dominance >=
            config->camera_ball.blue_dominance &&
        config->camera_ball.strong_blue_dominance <= 255 &&
        config->camera_ball.strong_blue_ratio_permille >=
            config->camera_ball.blue_ratio_permille &&
        config->camera_ball.strong_blue_ratio_permille <= 1000 &&
        config->camera_ball.minimum_strong_pixels > 0 &&
        config->camera_ball.minimum_strong_pixels <= 1000 &&
        config->camera_ball.minimum_strong_ratio_permille > 0 &&
        config->camera_ball.minimum_strong_ratio_permille <= 1000 &&
        config->camera_ball.minimum_mean_red_dominance > 0 &&
        config->camera_ball.minimum_mean_red_dominance <= 255 &&
        config->camera_ball.minimum_mean_blue_dominance > 0 &&
        config->camera_ball.minimum_mean_blue_dominance <= 255 &&
        config->camera_ball.minimum_area_permille > 0 &&
        config->camera_ball.minimum_area_permille <= 500 &&
        config->camera_ball.far_minimum_area_permille > 0 &&
        config->camera_ball.far_minimum_area_permille <
            config->camera_ball.minimum_area_permille &&
        config->camera_ball.far_minimum_confidence_permille > 0 &&
        config->camera_ball.far_minimum_confidence_permille <= 1000 &&
        config->camera_ball.minimum_fill_permille > 0 &&
        config->camera_ball.minimum_fill_permille <= 1000 &&
        config->camera_ball.minimum_roundness_permille > 0 &&
        config->camera_ball.minimum_roundness_permille <= 1000 &&
        config->camera_ball.minimum_blue_roundness_permille > 0 &&
        config->camera_ball.minimum_blue_roundness_permille <= 1000 &&
        config->camera_ball.blue_target_minimum_pixels > 1 &&
        config->camera_ball.blue_target_minimum_pixels <= 100 &&
        config->camera_ball.blue_target_minimum_strong_pixels > 0 &&
        config->camera_ball.blue_target_minimum_strong_pixels <=
            config->camera_ball.blue_target_minimum_pixels &&
        config->camera_ball.blue_target_minimum_mean_dominance > 0 &&
        config->camera_ball.blue_target_minimum_mean_dominance <=
            config->camera_ball.minimum_mean_blue_dominance &&
        config->camera_ball.blue_target_minimum_confidence_permille > 0 &&
        config->camera_ball.blue_target_minimum_confidence_permille <= 1000 &&
        config->camera_ball.blue_target_minimum_roundness_permille > 0 &&
        config->camera_ball.blue_target_minimum_roundness_permille <= 1000 &&
        config->camera_ball.highlight_minimum >= 0 &&
        config->camera_ball.highlight_minimum <= 255 &&
        config->camera_ball.highlight_max_chroma >= 0 &&
        config->camera_ball.highlight_max_chroma <= 255 &&
        config->camera_ball.highlight_target_tolerance >= 0 &&
        config->camera_ball.highlight_target_tolerance <= 255 &&
        config->camera_ball.highlight_expand_passes >= 0 &&
        config->camera_ball.highlight_expand_passes <= 4 &&
        config->camera_ball.edge_margin_pixels >= 0 &&
        config->camera_ball.edge_margin_pixels <= 10 &&
        config->camera_ball.tracking_tolerance_permille > 0 &&
        config->camera_ball.tracking_tolerance_permille <= 1000 &&
        config->camera_ball.confirm_frames > 0 &&
        config->camera_ball.confirm_frames <= 20 &&
        config->camera_ball.far_confirm_frames >=
            config->camera_ball.confirm_frames &&
        config->camera_ball.far_confirm_frames <= 20 &&
        config->ball_approach.target_center_x_permille >= -1000 &&
        config->ball_approach.target_center_x_permille <= 1000 &&
        config->ball_approach.route_deadband_permille > 0 &&
        config->ball_approach.route_deadband_permille <= 1000 &&
        valid_command(config->ball_approach.route_lateral_speed) &&
        config->ball_approach.route_lateral_speed > 0 &&
        config->ball_approach.route_pulse_ms > 0 &&
        config->ball_approach.route_confirm_frames > 0 &&
        config->ball_approach.route_align_timeout_ms > 0 &&
        config->ball_approach.align_deadband_permille > 0 &&
        config->ball_approach.realign_threshold_permille >
            config->ball_approach.align_deadband_permille &&
        config->ball_approach.realign_threshold_permille <= 1000 &&
        config->ball_approach.steering_gain_permille > 0 &&
        config->ball_approach.steering_gain_permille <= 2000 &&
        valid_command(config->ball_approach.maximum_correction) &&
        valid_command(config->ball_approach.search_speed) &&
        config->ball_approach.search_speed > 0 &&
        config->ball_approach.search_pulse_ms > 0 &&
        config->ball_approach.search_left_ms > 0 &&
        config->ball_approach.search_pulse_ms <=
            config->ball_approach.search_left_ms &&
        config->ball_approach.search_right_ms >=
            config->ball_approach.search_left_ms &&
        config->ball_approach.search_settle_ms > 0 &&
        valid_command(config->ball_approach.goal_search_speed) &&
        config->ball_approach.goal_search_speed > 0 &&
        config->ball_approach.acquire_timeout_ms > 0 &&
        valid_command(config->ball_approach.align_speed) &&
        config->ball_approach.align_speed > 0 &&
        config->ball_approach.align_pulse_ms > 0 &&
        config->ball_approach.align_settle_ms > 0 &&
        config->ball_approach.align_confirm_frames > 0 &&
        valid_command(config->ball_approach.far_forward_speed) &&
        valid_command(config->ball_approach.medium_forward_speed) &&
        valid_command(config->ball_approach.near_forward_speed) &&
        valid_command(config->ball_approach.forward_boost_speed) &&
        config->ball_approach.far_forward_speed > 0 &&
        config->ball_approach.medium_forward_speed > 0 &&
        config->ball_approach.near_forward_speed > 0 &&
        config->ball_approach.forward_boost_speed >=
            config->ball_approach.far_forward_speed &&
        config->ball_approach.forward_boost_ms >= 0 &&
        config->ball_approach.medium_y_permille > 0 &&
        config->ball_approach.medium_y_permille <
            config->ball_approach.near_y_permille &&
        config->ball_approach.near_y_permille <
            config->ball_approach.capture_center_y_permille &&
        config->ball_approach.capture_center_y_permille <= 1000 &&
        config->ball_approach.capture_height_permille > 0 &&
        config->ball_approach.capture_height_permille <= 1000 &&
        config->ball_approach.capture_area_permille > 0 &&
        config->ball_approach.capture_area_permille <= 1000 &&
        config->ball_approach.capture_box_bottom_permille > 0 &&
        config->ball_approach.capture_box_bottom_permille <= 1000 &&
        config->ball_approach.capture_confirm_frames > 0 &&
        config->ball_approach.capture_verify_max_frames >=
            config->ball_approach.capture_confirm_frames &&
        config->ball_approach.capture_verify_max_frames <= 20 &&
        config->ball_approach.maximum_approach_ms > 0 &&
        valid_command(config->ball_approach.push_speed) &&
        config->ball_approach.push_speed > 0 &&
        valid_command(config->ball_approach.push_boost_speed) &&
        config->ball_approach.push_boost_speed >=
            config->ball_approach.push_speed &&
        config->ball_approach.push_boost_ms > 0 &&
        config->ball_approach.push_boost_ms <= 1000 &&
        config->ball_approach.push_steering_gain_permille > 0 &&
        config->ball_approach.push_steering_gain_permille <= 2000 &&
        valid_command(config->ball_approach.push_maximum_correction) &&
        config->ball_approach.push_realign_threshold_permille >
            config->ball_approach.align_deadband_permille &&
        config->ball_approach.push_realign_threshold_permille <= 1000 &&
        config->ball_approach.push_timeout_ms > 0 &&
        config->ball_approach.goal_overlap_margin_permille >= 0 &&
        config->ball_approach.goal_overlap_margin_permille <= 300 &&
        config->ball_approach.goal_overlap_confirm_frames > 0 &&
        config->ball_approach.maximum_total_ms >=
            config->ball_approach.maximum_approach_ms +
            config->ball_approach.push_timeout_ms &&
        config->ball_approach.recovery_wait_ms >= 500 &&
        config->ball_approach.recovery_wait_ms <= 10000 &&
        config->button.press_debounce_ms > 0 &&
        config->button.release_rearm_ms > 0 &&
        config->button.startup_guard_ms >= 0 &&
        config->default_speed >= 100 && config->default_speed <= 800;
}
