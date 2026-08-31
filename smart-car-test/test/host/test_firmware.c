#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "camera_line_vision.h"
#include "driver/gpio.h"
#include "kiwi_kinematics.h"
#include "line_follow.h"
#include "motor_driver.h"
#include "obstacle_supervisor.h"
#include "start_button.h"
#include "ultrasonic.h"

typedef enum {
    HAL_EVENT_PREINIT,
    HAL_EVENT_INIT,
    HAL_EVENT_ENABLE,
    HAL_EVENT_DIRECTION,
    HAL_EVENT_DUTY,
} hal_event_kind_t;

typedef struct {
    hal_event_kind_t kind;
    int channel;
    int value;
} hal_event_t;

static hal_event_t s_hal_events[64];
static size_t s_hal_event_count;
static hal_event_kind_t s_fail_kind;
static int s_gpio_levels[64];
static void (*s_gpio_isr)(void *);
static void *s_gpio_isr_arg;
static int64_t s_fake_time_us;

static bool record_hal(hal_event_kind_t kind, int channel, int value)
{
    assert(s_hal_event_count < sizeof(s_hal_events) / sizeof(s_hal_events[0]));
    s_hal_events[s_hal_event_count++] = (hal_event_t) {kind, channel, value};
    if (s_fail_kind == kind) {
        s_fail_kind = (hal_event_kind_t)-1;
        return false;
    }
    return true;
}

static void reset_hal(void)
{
    memset(s_hal_events, 0, sizeof(s_hal_events));
    s_hal_event_count = 0;
    s_fail_kind = (hal_event_kind_t)-1;
}

bool motor_hal_preinit_safe(const motor_hal_config_t *config)
{
    (void)config;
    return record_hal(HAL_EVENT_PREINIT, -1, 0);
}

bool motor_hal_init_outputs(const motor_hal_config_t *config)
{
    (void)config;
    return record_hal(HAL_EVENT_INIT, -1, 0);
}

bool motor_hal_set_enable(const motor_hal_config_t *config, bool enabled)
{
    (void)config;
    return record_hal(HAL_EVENT_ENABLE, -1, enabled);
}

bool motor_hal_set_direction(const motor_hal_config_t *config,
                             int channel, int direction)
{
    (void)config;
    return record_hal(HAL_EVENT_DIRECTION, channel, direction);
}

bool motor_hal_set_duty(const motor_hal_config_t *config,
                        int channel, uint32_t duty)
{
    (void)config;
    return record_hal(HAL_EVENT_DUTY, channel, (int)duty);
}

esp_err_t gpio_config(const gpio_config_t *config)
{
    (void)config;
    return ESP_OK;
}

int gpio_get_level(int pin)
{
    assert(pin >= 0 && pin < (int)(sizeof(s_gpio_levels) /
                                  sizeof(s_gpio_levels[0])));
    return s_gpio_levels[pin];
}

esp_err_t gpio_set_level(int pin, int level)
{
    assert(pin >= 0 && pin < (int)(sizeof(s_gpio_levels) /
                                  sizeof(s_gpio_levels[0])));
    s_gpio_levels[pin] = level;
    return ESP_OK;
}

esp_err_t gpio_isr_handler_add(int pin, void (*handler)(void *), void *arg)
{
    (void)pin;
    s_gpio_isr = handler;
    s_gpio_isr_arg = arg;
    return ESP_OK;
}

int64_t esp_timer_get_time(void)
{
    return s_fake_time_us;
}

void esp_rom_delay_us(uint32_t microseconds)
{
    s_fake_time_us += microseconds;
}

static line_sensor_sample_t line(bool left, bool left_center,
                                 bool right_center, bool right)
{
    return (line_sensor_sample_t) {left, left_center, right_center, right};
}

enum {
    TEST_IMAGE_WIDTH = 160,
    TEST_IMAGE_HEIGHT = 120,
};

static uint8_t s_test_image[TEST_IMAGE_WIDTH * TEST_IMAGE_HEIGHT * 3];
static camera_line_vision_workspace_t s_camera_line_workspace;

static void fill_test_image(uint8_t gray)
{
    memset(s_test_image, gray, sizeof(s_test_image));
}

static void draw_view_rectangle(int left, int top, int right, int bottom,
                                uint8_t gray)
{
    for (int y = top; y <= bottom; ++y) {
        for (int x = left; x <= right; ++x) {
            uint8_t *pixel = &s_test_image[
                (y * TEST_IMAGE_WIDTH + x) * 3];
            pixel[0] = gray;
            pixel[1] = gray;
            pixel[2] = gray;
        }
    }
}

static camera_line_analysis_t analyze_test_image(void)
{
    camera_line_config_t config = APP_CONFIG.camera_line;
    /* Test geometric centering independently of installation calibration. */
    config.center_offset_permille = 0;
    return camera_line_analyze_lower_half_rgb888(
        s_test_image, TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT, false,
        &config, &s_camera_line_workspace);
}

static camera_line_analysis_t analyze_test_image_with_hint(
    int previous_center_permille, int previous_steering_permille)
{
    camera_line_config_t config = APP_CONFIG.camera_line;
    config.center_offset_permille = 0;
    return camera_line_analyze_lower_half_rgb888_with_hint(
        s_test_image, TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT, false,
        &config, &s_camera_line_workspace, true,
        previous_center_permille, previous_steering_permille);
}

static void test_camera_line_vision(void)
{
    assert(APP_CONFIG.camera_line.center_offset_permille == -165);
    assert(APP_CONFIG.camera_line.roi_left_permille == 250);
    assert(APP_CONFIG.camera_line.roi_right_permille == 750);
    assert(APP_CONFIG.camera_line.horizontal_scale_permille == 1000);
    assert(APP_CONFIG.camera_line.hairpin_near_threshold_permille == 160);
    assert(APP_CONFIG.camera_line.hairpin_heading_threshold_permille == 300);
    assert(APP_CONFIG.camera_line.hairpin_heading_gain_permille == 100);
    /* The captured hairpin geometry must not cancel into an almost-straight
     * steering request. */
    assert(camera_line_steering_from_geometry(
               250, -160, &APP_CONFIG.camera_line) == 209);
    assert(camera_line_steering_from_geometry(
               -250, 160, &APP_CONFIG.camera_line) == -209);
    assert(camera_line_steering_from_geometry(
               250, 100, &APP_CONFIG.camera_line) == 175);
    assert(line_sensor_pattern(camera_line_virtual_sensors(-601, false)) ==
           0x08);
    assert(line_sensor_pattern(camera_line_virtual_sensors(-600, false)) ==
           0x04);
    assert(line_sensor_pattern(camera_line_virtual_sensors(-200, false)) ==
           0x06);
    assert(line_sensor_pattern(camera_line_virtual_sensors(200, false)) ==
           0x06);
    assert(line_sensor_pattern(camera_line_virtual_sensors(201, false)) ==
           0x02);
    assert(line_sensor_pattern(camera_line_virtual_sensors(601, false)) ==
           0x01);

    fill_test_image(225);
    camera_line_analysis_t analysis = analyze_test_image();
    assert(analysis.valid && !analysis.line_detected);

    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected);
    assert(!analysis.finish_detected);
    assert(line_sensor_pattern(analysis.virtual_sensors) == 0x06);
    assert(analysis.center_permille == 0);
    assert(analysis.far_center_permille == 0);
    assert(analysis.heading_permille == 0);
    assert(analysis.steering_permille == 0);
    assert(analysis.connected_component_count == 1);
    assert(analysis.component_height_permille >= 900);

    fill_test_image(225);
    draw_view_rectangle(41, 72, 49, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected &&
           analysis.center_permille < -500);
    assert(line_sensor_pattern(analysis.virtual_sensors) == 0x08);

    fill_test_image(225);
    draw_view_rectangle(110, 72, 118, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected &&
           analysis.center_permille > 500);
    assert(line_sensor_pattern(analysis.virtual_sensors) == 0x01);

    /* The restored horizontal crop rejects a physical-edge dark region even
     * when an old steering hint points toward it. */
    fill_test_image(225);
    draw_view_rectangle(4, 72, 15, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && !analysis.line_detected);
    analysis = analyze_test_image_with_hint(-800, -800);
    assert(analysis.valid && !analysis.line_detected);

    fill_test_image(225);
    draw_view_rectangle(40, 90, 119, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.line_detected && analysis.finish_detected);
    assert(line_sensor_pattern(analysis.virtual_sensors) == 0x0f);

    /* Dark objects outside the central track window must not beat the line. */
    fill_test_image(225);
    draw_view_rectangle(0, 72, 35, 110, 5);
    draw_view_rectangle(124, 72, 159, 110, 5);
    draw_view_rectangle(74, 72, 85, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.line_detected && !analysis.finish_detected);
    assert(line_sensor_pattern(analysis.virtual_sensors) == 0x06);
    assert(analysis.center_permille == 0);

    /* The upper half remains structurally outside the detector. */
    fill_test_image(225);
    draw_view_rectangle(0, 0, TEST_IMAGE_WIDTH - 1,
                        TEST_IMAGE_HEIGHT / 2 - 1, 5);
    analysis = analyze_test_image();
    assert(analysis.valid && !analysis.line_detected);

    /* Separate islands remain separate components.  Without reliable history,
     * the largest connected island is still accepted. */
    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 75, 25);
    draw_view_rectangle(74, 88, 85, 91, 25);
    draw_view_rectangle(74, 107, 85, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected);
    assert(analysis.connected_component_count == 3);

    /* Eight-neighbour connectivity preserves a diagonal track after the
     * camera's 1/8-scale decode. */
    fill_test_image(225);
    for (int y = 72; y <= 110; ++y) {
        const int x = 65 + (y - 72) / 2;
        draw_view_rectangle(x, y, x + 3, y, 25);
    }
    analysis = analyze_test_image();
    assert(analysis.line_detected && !analysis.finish_detected);
    assert(analysis.heading_permille < 0);
    assert(analysis.steering_permille < analysis.center_permille);

    /* A thin connected line may travel across most of the ROI after only a
     * short straight approach.  Its local row thickness remains track-like,
     * and the centered near end must allow initial acquisition even though
     * the lookahead steering is already a hard left turn. */
    fill_test_image(225);
    for (int y = 96; y <= 110; ++y) {
        const int x = 40 + (y - 96) * 40 / 14;
        draw_view_rectangle(x, y, x + 4, y, 25);
    }
    analysis = analyze_test_image();
    assert(analysis.line_detected && !analysis.finish_detected);
    assert(analysis.center_permille > -400 &&
           analysis.center_permille < 400);
    assert(analysis.steering_permille < -400);
    assert(analysis.width_permille < 100);

    /* Width is diagnostic only: a connected object is accepted even when it
     * exceeds the retired normal-track width limit. */
    fill_test_image(225);
    draw_view_rectangle(55, 72, 104, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected);

    /* A matching history can select a smaller connected track over a larger
     * disconnected dark region.  The same image without history keeps the
     * original largest-component fallback. */
    fill_test_image(225);
    draw_view_rectangle(41, 72, 60, 110, 25);
    draw_view_rectangle(108, 72, 116, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected &&
           analysis.connected_component_count == 2 &&
           analysis.center_permille < -400);
    analysis = analyze_test_image_with_hint(700, 700);
    assert(analysis.valid && analysis.line_detected &&
           analysis.connected_component_count == 2 &&
           analysis.center_permille > 500);

    /* Soft history is not a jump gate: a sole component on the opposite side
     * remains valid even when both historical terms strongly disagree. */
    fill_test_image(225);
    draw_view_rectangle(110, 72, 118, 110, 25);
    camera_line_config_t hinted_config = APP_CONFIG.camera_line;
    hinted_config.center_offset_permille = 0;
    analysis = camera_line_analyze_lower_half_rgb888_with_hint(
        s_test_image, TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT, false,
        &hinted_config, &s_camera_line_workspace, true, -800, -800);
    assert(analysis.valid && analysis.line_detected &&
           analysis.center_permille > 500);
}

static ultrasonic_event_t ultrasonic(uint32_t seq, bool has_echo,
                                     int raw_mm, int filtered_mm,
                                     ultrasonic_quality_t quality,
                                     bool echo_high, bool uncertain)
{
    return (ultrasonic_event_t) {
        .seq = seq,
        .has_echo = has_echo,
        .raw_mm = raw_mm,
        .filtered_mm = filtered_mm,
        .quality = quality,
        .echo_high = echo_high,
        .safety_uncertain = uncertain,
    };
}

static obstacle_decision_t obstacle_step(obstacle_supervisor_t *supervisor,
                                         const ultrasonic_event_t *event)
{
    s_fake_time_us += 20000;
    return obstacle_supervisor_step(
        supervisor, event, line(false, false, false, false), s_fake_time_us);
}

static obstacle_decision_t obstacle_step_at(
    obstacle_supervisor_t *supervisor, const ultrasonic_event_t *event,
    line_sensor_sample_t sensors, int64_t now_us)
{
    return obstacle_supervisor_step(supervisor, event, sensors, now_us);
}

static void test_kiwi_kinematics(void)
{
    const kiwi_kinematics_config_t ideal = {
        .lateral_side_permille = 866,
        .lateral_yaw_compensation_permille = 0,
        .right_lateral_yaw_compensation_permille = 0,
        .lateral_side_wheel_minimum = 0,
        .motor_b_positive_minimum = 0,
    };
    motor_command_t command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.forward = 400}, &ideal);
    assert(command.a == -400 && command.b == 0 && command.c == -400);

    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.clockwise = 420}, &ideal);
    assert(command.a == 420 && command.b == -420 && command.c == -420);

    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = 400}, &ideal);
    assert(command.a == -346 && command.b == -400 && command.c == 346);
    const motor_command_t right = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -400}, &ideal);
    assert(right.a == -command.a && right.b == -command.b &&
           right.c == -command.c);

    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {
            .forward = -1000, .left = 1000, .clockwise = 1000,
        }, &ideal);
    assert(command.a == 567);
    assert(command.b == -1000);
    assert(command.c == 433);

    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = 380}, &APP_CONFIG.kinematics);
    assert(APP_CONFIG.kinematics.lateral_yaw_compensation_permille == 500);
    assert(command.a == -300 && command.b == -570 && command.c == 300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = 500}, &APP_CONFIG.kinematics);
    assert(command.a == -300 && command.b == -750 && command.c == 300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -380}, &APP_CONFIG.kinematics);
    assert(APP_CONFIG.kinematics.right_lateral_yaw_compensation_permille ==
           500);
    assert(command.a == 300 && command.b == 570 && command.c == -300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -500}, &APP_CONFIG.kinematics);
    assert(command.a == 300 && command.b == 750 && command.c == -300);
}

static void test_line_follow_behavior(void)
{
    assert(APP_CONFIG.line.straight_speed == 348);
    assert(APP_CONFIG.line.curve_speed == 276);
    assert(APP_CONFIG.line.curve_max == 384);
    assert(APP_CONFIG.line.edge_speed == 233);
    assert(APP_CONFIG.line.edge_max == 336);
    assert(APP_CONFIG.line.search_speed == 211);
    assert(APP_CONFIG.line.kp == 120);
    assert(APP_CONFIG.line.max_correction == 300);
    assert(APP_CONFIG.line.direction_confirm_count == 3);
    assert(APP_CONFIG.line.direction_hold_error == 4);
    assert(APP_CONFIG.line.single_sensor_inner_command == 100);
    assert(APP_CONFIG.line.drive_assist_threshold == 200);
    assert(APP_CONFIG.line.drive_assist_command == 500);
    assert(APP_CONFIG.line.drive_assist_ms == 150);
    assert(APP_CONFIG.line.turn_memory_threshold_permille == 180);
    assert(APP_CONFIG.line.turn_memory_release_permille == 80);
    assert(APP_CONFIG.line.turn_memory_ms == 180);
    assert(APP_CONFIG.line.lost_motion_memory_ms == 120);
    assert(APP_CONFIG.line.lost_search_delay_ms == 150);
    assert(APP_CONFIG.line.search_direction_threshold_permille == 100);
    assert(APP_CONFIG.line.search_primary_ms == 1200);
    assert(APP_CONFIG.line.search_reverse_ms == 2400);
    assert(APP_CONFIG.line.search_max_sweep_ms == 7200);
    assert(APP_CONFIG.line.search_reacquire_frames == 3);
    assert(APP_CONFIG.default_speed == 400);

    line_follow_t controller;
    line_follow_init(&controller, &APP_CONFIG.line);
    line_follow_reset_for_start(&controller, line(false, true, true, false));

    motor_command_t command = line_follow_step(
        &controller, line(false, true, true, false), 400, 0);
    assert(command.a == -500 && command.b == 0 && command.c == -500);
    command = line_follow_step(
        &controller, line(false, true, true, false), 400, 149999);
    assert(command.a == -500 && command.c == -500);
    command = line_follow_step(
        &controller, line(false, true, true, false), 400, 150000);
    assert(command.a == -348 && command.c == -348);

    for (int index = 0; index < 3; ++index) {
        command = line_follow_step(
            &controller, line(false, false, false, true), 400,
            200000 + index * 20000);
        assert(command.a == 100 && command.b == 0 && command.c == -336);
    }
    assert(controller.locked_direction == 1);
    assert(controller.error == 6 && controller.control_error == 6);
    command = line_follow_step(&controller, line(false, true, false, false),
                               400, 260000);
    assert(command.a == 42 && command.b == 0 && command.c == -336);
    assert(controller.locked_direction == 1);
    assert(controller.control_error == 4);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 300000);
    assert(command.a == 42 && command.b == 0 && command.c == -336);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 460000);
    assert(command.a == 500 && command.b == 0 && command.c == -500);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 620000);
    assert(command.a == 211 && command.b == 0 && command.c == -211);

    /* Search stops on the first candidate and requires three distinct camera
     * frames before forward line following resumes. */
    command = line_follow_step_camera(
        &controller, line(false, false, true, false), true, 300,
        10, 400, 640000);
    assert(command.a == 0 && command.c == 0);
    assert(controller.state == LINE_STATE_REACQUIRE &&
           controller.reacquire_count == 1);
    command = line_follow_step_camera(
        &controller, line(false, false, true, false), true, 300,
        10, 400, 660000);
    assert(command.a == 0 && command.c == 0 &&
           controller.reacquire_count == 1);
    command = line_follow_step_camera(
        &controller, line(false, false, true, false), true, 300,
        11, 400, 720000);
    assert(command.a == 0 && command.c == 0 &&
           controller.reacquire_count == 2);
    command = line_follow_step_camera(
        &controller, line(false, false, true, false), true, 300,
        12, 400, 800000);
    assert(command.a == -72 && command.c == -500);

    line_follow_reset_for_start(&controller, line(true, false, false, false));
    command = line_follow_step(&controller, line(true, false, false, false),
                               400, 0);
    assert(command.a == -500 && command.b == 0 && command.c == 100);
    command = line_follow_step(&controller, line(true, false, false, false),
                               400, 150000);
    assert(command.a == -336 && command.c == 100);

    line_follow_reset_for_start(&controller, line(false, true, true, true));
    command = line_follow_step(&controller, line(false, true, true, true),
                               400, 0);
    assert(command.a == -26 && command.b == 0 && command.c == -500);

    command = line_follow_step(&controller, line(false, true, true, true),
                               400, 150000);
    assert(command.a == -26 && command.b == 0 && command.c == -384);

    command = line_follow_step(&controller, line(true, false, true, false),
                               400, 170000);
    assert(command.a == -500 && command.b == 0 && command.c == -26);

    line_follow_reset_for_start(&controller, line(false, false, true, false));
    command = line_follow_step(&controller, line(false, false, true, false),
                               400, 0);
    assert(command.a == 100 && command.b == 0 && command.c == -500);
    command = line_follow_step(&controller, line(false, false, true, false),
                               400, 150000);
    assert(command.a == 100 && command.b == 0 && command.c == -336);

    line_follow_reset_for_start(&controller, line(false, true, false, false));
    command = line_follow_step(&controller, line(false, true, false, false),
                               400, 0);
    assert(command.a == -500 && command.b == 0 && command.c == 100);
    command = line_follow_step(&controller, line(false, true, false, false),
                               400, 150000);
    assert(command.a == -336 && command.b == 0 && command.c == 100);

    line_follow_reset_for_start(&controller, line(false, false, false, false));
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 0);
    assert(command.a == 0 && command.b == 0 && command.c == 0);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 150000);
    assert(command.a == -500 && command.b == 0 && command.c == 500);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 299999);
    assert(command.a == -500 && command.b == 0 && command.c == 500);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 300000);
    assert(command.a == -211 && command.b == 0 && command.c == 211);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 1349999);
    assert(command.a == -211 && command.c == 211);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 1350000);
    assert(command.a == 500 && command.c == -500);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 1499999);
    assert(command.a == 500 && command.c == -500);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 1500000);
    assert(command.a == 211 && command.c == -211);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 3749999);
    assert(command.a == 211 && command.c == -211);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 3750000);
    assert(command.a == -500 && command.c == 500);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 3899999);
    assert(command.a == -500 && command.c == 500);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 3900000);
    assert(command.a == -211 && command.c == 211);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 7349999);
    assert(command.a == -211 && command.c == 211);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 7350000);
    assert(command.a == 500 && command.c == -500);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 12149999);
    assert(command.a == 211 && command.c == -211);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 12150000);
    assert(command.a == -500 && command.c == 500);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 18149999);
    assert(command.a == -211 && command.c == 211);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 18150000);
    assert(command.a == 500 && command.c == -500);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 25349999);
    assert(command.a == 211 && command.c == -211);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 25350000);
    assert(command.a == -500 && command.c == 500);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 32549999);
    assert(command.a == -211 && command.c == 211);
    assert(controller.state == LINE_STATE_SEARCH_LEFT);
    command = line_follow_step(&controller, line(false, false, false, false),
                               400, 32550000);
    assert(command.a == 500 && command.c == -500);
    assert(controller.state == LINE_STATE_SEARCH_RIGHT);

    /* With no discrete direction lock, the last meaningful camera steering
     * sign selects the primary search direction. */
    line_follow_reset_for_start(&controller, line(false, true, true, false));
    command = line_follow_step_camera(
        &controller, line(false, true, true, false), true, 400,
        20, 400, 0);
    assert(controller.locked_direction == 0 &&
           controller.last_steering_direction == 1);
    command = line_follow_step_camera(
        &controller, line(false, false, false, false), false, 0,
        21, 400, 100000);
    assert(command.a == -148 && command.c == -548 &&
           controller.state == LINE_STATE_SEARCH_RIGHT);
    command = line_follow_step_camera(
        &controller, line(false, false, false, false), false, 0,
        22, 400, 250000);
    assert(command.a == 500 && command.c == -500);

    /* Camera steering retains sub-band position instead of collapsing every
     * right-center observation to the same discrete correction. */
    line_follow_reset_for_start(&controller, line(false, false, true, false));
    command = line_follow_step_camera(
        &controller, line(false, false, true, false), true, 300,
        30, 400, 0);
    assert(command.a == -72 && command.b == 0 && command.c == -500);
    command = line_follow_step_camera(
        &controller, line(false, false, true, false), true, 300,
        31, 400, 150000);
    assert(command.a == -72 && command.b == 0 && command.c == -336);

    /* A newly established camera turn keeps the previous trajectory briefly,
     * then applies the current turn.  A directly following loss reuses that
     * last proven cruise command for only the configured blind-zone memory. */
    line_follow_reset_for_start(&controller, line(false, true, true, false));
    command = line_follow_step_camera(
        &controller, line(false, true, true, false), true, 0,
        40, 400, 0);
    assert(command.a == -500 && command.c == -500);
    command = line_follow_step_camera(
        &controller, line(false, true, true, false), true, 400,
        41, 400, 200000);
    assert(command.a == -348 && command.c == -348);
    command = line_follow_step_camera(
        &controller, line(false, true, true, false), true, 400,
        42, 400, 379999);
    assert(command.a == -348 && command.c == -348);
    command = line_follow_step_camera(
        &controller, line(false, true, true, false), true, 400,
        43, 400, 380000);
    assert(command.a == -148 && command.c == -548);
    command = line_follow_step_camera(
        &controller, line(false, false, false, false), false, 0,
        44, 400, 400000);
    assert(command.a == -148 && command.c == -548);
    command = line_follow_step_camera(
        &controller, line(false, false, false, false), false, 0,
        45, 400, 519999);
    assert(command.a == -148 && command.c == -548);
    command = line_follow_step_camera(
        &controller, line(false, false, false, false), false, 0,
        46, 400, 520000);
    assert(command.a == 0 && command.c == 0);
    command = line_follow_step_camera(
        &controller, line(false, false, false, false), false, 0,
        47, 400, 550000);
    assert(command.a == 500 && command.c == -500);

    line_follow_suspend(&controller);
    line_follow_resume(&controller);
    command = line_follow_step(&controller, line(false, true, true, false),
                               400, 200000);
    assert(command.a == -500 && command.b == 0 && command.c == -500);
}

static void test_obstacle_supervisor(void)
{
    obstacle_config_t config = APP_CONFIG.obstacle;
    assert(config.stop_mm == 100);
    assert(config.no_echo_limit == 3);
    assert(APP_CONFIG.ultrasonic.timeout_us == 45000);
    assert(APP_CONFIG.ultrasonic.period_ms == 70);
    assert(config.lateral_speed == 380);
    assert(config.lateral_start_speed == 500);
    assert(config.left_strafe_ms == 1231);
    assert(config.forward_drive_ms == 1191);
    assert(config.right_strafe_ms == 960);
    config.bypass_enabled = false;
    obstacle_supervisor_t supervisor;
    obstacle_supervisor_init(&supervisor, &config);

    /* Open-space no-Echo/timeout samples authorize normal startup. */
    ultrasonic_event_t event = ultrasonic(
        1, false, -1, -1, ULTRASONIC_QUALITY_OUTLIER, false, false);
    obstacle_decision_t decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(supervisor.state == OBSTACLE_STATE_SENSOR_CHECK);
    event.seq = 2;
    event.quality = ULTRASONIC_QUALITY_LOST;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    event.seq = 3;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_CLEAR);

    /* Valid far observations remain an equivalent startup path. */
    obstacle_supervisor_reset(&supervisor);
    event = ultrasonic(
        4, true, 300, 300, ULTRASONIC_QUALITY_VALID, false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK && decision.clear_count == 1);
    event.seq = 5;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK && decision.clear_count == 2);
    event.seq = 6;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_CLEAR);

    event = ultrasonic(7, true, 101, 101, ULTRASONIC_QUALITY_VALID,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);

    /* Open space, completed no-return pulses and clean jumps keep running. */
    event = ultrasonic(8, true, 6500, 101, ULTRASONIC_QUALITY_NO_RETURN,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(supervisor.no_echo_count == 0);
    event.seq = 9;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    event = ultrasonic(10, false, -1, 101, ULTRASONIC_QUALITY_LOST,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    event = ultrasonic(11, true, 1324, 775, ULTRASONIC_QUALITY_OUTLIER,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);

    /* A near raw Echo still stops immediately, even if it is an outlier. */
    event = ultrasonic(12, true, 100, 300, ULTRASONIC_QUALITY_OUTLIER,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_WAIT_CLEAR);
    assert(decision.reason == OBSTACLE_REASON_NEAR);
    assert(decision.line_action == LINE_ACTION_SUSPEND);

    /* Once a near object disappears, open-space samples release WAIT_CLEAR. */
    event = ultrasonic(13, false, -1, 101, ULTRASONIC_QUALITY_OUTLIER,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.clear_count == 1);
    event = ultrasonic(14, true, 6500, 101,
                       ULTRASONIC_QUALITY_NO_RETURN, false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.clear_count == 2);
    event = ultrasonic(15, false, -1, 101, ULTRASONIC_QUALITY_LOST,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.reason == OBSTACLE_REASON_THREE_CLEAR);
    assert(decision.line_action == LINE_ACTION_RESUME);

    /* Other ultrasonic faults must not interrupt active line following. */
    event = ultrasonic(16, true, 350, 350, ULTRASONIC_QUALITY_INVALID,
                       false, true);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
}

static void test_automatic_bypass_sequence(void)
{
    obstacle_config_t config = APP_CONFIG.obstacle;
    config.bypass_enabled = true;
    obstacle_supervisor_t supervisor;
    obstacle_supervisor_init(&supervisor, &config);
    line_sensor_sample_t white = line(false, false, false, false);
    line_sensor_sample_t black = line(false, true, false, false);
    ultrasonic_event_t event;
    obstacle_decision_t decision = {0};

    for (uint32_t seq = 1; seq <= 3; ++seq) {
        event = ultrasonic(seq, true, 300, 300,
                           ULTRASONIC_QUALITY_VALID, false, false);
        decision = obstacle_step_at(&supervisor, &event, white,
                                    seq * 60000LL);
    }
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);

    /* 1111 is ordinary track before an avoidance has completed. */
    line_sensor_sample_t all_black = line(true, true, true, true);
    decision = obstacle_step_at(&supervisor, NULL, all_black, 190000);
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);

    event = ultrasonic(4, true, 100, 300,
                       ULTRASONIC_QUALITY_OUTLIER, false, false);
    decision = obstacle_step_at(&supervisor, &event, white, 200000);
    assert(supervisor.state == OBSTACLE_STATE_BRAKE);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.line_action == LINE_ACTION_SUSPEND);

    decision = obstacle_step_at(&supervisor, NULL, white,
                                200000 + config.brake_ms * 1000LL);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_LEFT_DISTANCE);
    assert(decision.policy == MOTION_POLICY_OVERRIDE);
    assert(decision.override_motion.left == config.lateral_start_speed);

    int64_t now_us = supervisor.phase_started_us +
                     config.left_strafe_ms * 1000LL - 1;
    decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_LEFT_DISTANCE);
    assert(decision.override_motion.left == config.lateral_speed);

    now_us++;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_SETTLE_FORWARD);
    assert(decision.policy == MOTION_POLICY_BLOCK);

    now_us += config.brake_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_FORWARD_DISTANCE);
    assert(decision.override_motion.forward == config.forward_start_speed);

    now_us += config.forward_drive_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_SETTLE_RIGHT);
    assert(decision.policy == MOTION_POLICY_BLOCK);

    now_us += config.brake_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_RIGHT_DISTANCE);
    assert(decision.override_motion.left == -config.lateral_start_speed);

    /* Missing the line at the end of the right strafe resumes line search
     * instead of entering a latched fail-safe stop. */
    obstacle_supervisor_t timeout_supervisor = supervisor;
    decision = obstacle_step_at(
        &timeout_supervisor, NULL, white,
        timeout_supervisor.phase_started_us +
            config.right_strafe_ms * 1000LL);
    assert(timeout_supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.reason == OBSTACLE_REASON_LINE_LOST);
    assert(decision.line_action == LINE_ACTION_RESUME);

    /* The first black sample stops right strafe immediately. */
    now_us += config.right_strafe_ms * 500LL;
    decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_LINE_CONFIRM);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.reason == OBSTACLE_REASON_LINE_SEEN);

    /* A candidate that disappears during confirmation follows the same
     * recoverable line-search path. */
    obstacle_supervisor_t confirmation_loss_supervisor = supervisor;
    decision = obstacle_step_at(&confirmation_loss_supervisor, NULL, white,
                                now_us + 20000);
    assert(confirmation_loss_supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.reason == OBSTACLE_REASON_LINE_LOST);
    assert(decision.line_action == LINE_ACTION_RESUME);

    for (int count = 1; count < config.line_confirm_count; ++count) {
        now_us += 20000;
        decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    }
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.line_action == LINE_ACTION_RESUME);
    assert(supervisor.bypass_completed);

    /* After avoidance, 1111 is a latched finish-line stop. */
    now_us += 20000;
    decision = obstacle_step_at(&supervisor, NULL, all_black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_FINISHED);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_FINISHED);
    assert(decision.reason == OBSTACLE_REASON_FINISH_LINE);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.line_action == LINE_ACTION_SUSPEND);
    now_us += 20000;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_FINISHED);
    assert(decision.policy == MOTION_POLICY_BLOCK);
}

static void test_start_button(void)
{
    start_button_t button;
    const start_button_config_t config = APP_CONFIG.button;
    s_gpio_levels[0] = 1;
    assert(start_button_init(&button, 0, &config, 0) == ESP_OK);
    assert(!start_button_update_level(&button, true, 100000));
    assert(!start_button_update_level(&button, true, 149000));
    assert(!start_button_update_level(&button, true, 150000));
    assert(!start_button_update_level(&button, false, 200000));
    assert(!start_button_update_level(&button, false, 250000));
    assert(start_button_update_level(&button, false, 500000));
    assert(button.armed);
    assert(!start_button_update_level(&button, true, 600000));
    assert(start_button_update_level(&button, true, 650000));
    assert(!start_button_update_level(&button, true, 800000));

    s_gpio_levels[0] = 0;
    assert(start_button_init(&button, 0, &config, 0) == ESP_OK);
    assert(!button.armed);
    assert(!start_button_update_level(&button, false, 100000));
    assert(!start_button_update_level(&button, false, 150000));
    assert(!button.armed);
    assert(!start_button_update_level(&button, false, 230000));
    assert(button.armed);
    assert(!start_button_update_level(&button, true, 600000));
    assert(start_button_update_level(&button, true, 650000));
}

static void ultrasonic_edge(int echo_pin, int level, int64_t now_us)
{
    assert(s_gpio_isr != NULL);
    s_gpio_levels[echo_pin] = level;
    s_fake_time_us = now_us;
    s_gpio_isr(s_gpio_isr_arg);
}

static void test_ultrasonic_transactions(void)
{
    const int trigger_pin = 6;
    const int echo_pin = 13;
    ultrasonic_t sensor;
    const ultrasonic_driver_config_t config = {
        .trigger_pin = trigger_pin,
        .echo_pin = echo_pin,
        .timing = APP_CONFIG.ultrasonic,
    };
    memset(s_gpio_levels, 0, sizeof(s_gpio_levels));
    s_gpio_isr = NULL;
    s_gpio_isr_arg = NULL;
    s_fake_time_us = 0;
    assert(ultrasonic_init(&sensor, &config, 0) == ESP_OK);
    ultrasonic_step(&sensor, 0);
    assert(sensor.active);

    ultrasonic_edge(echo_pin, 1, 1000);
    ultrasonic_edge(echo_pin, 0, 1584);
    ultrasonic_step(&sensor, 46000);
    ultrasonic_event_t event;
    assert(ultrasonic_take_event(&sensor, &event));
    assert(event.has_echo && event.raw_mm == 100);
    assert(event.quality == ULTRASONIC_QUALITY_VALID);

    s_fake_time_us = 70000;
    ultrasonic_step(&sensor, 70000);
    ultrasonic_edge(echo_pin, 1, 71000);
    ultrasonic_edge(echo_pin, 0, 72750);
    ultrasonic_edge(echo_pin, 1, 72800);
    ultrasonic_step(&sensor, 116000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(event.has_echo && event.pulse_us == 1750);
    assert(event.quality == ULTRASONIC_QUALITY_INVALID);
    assert(event.safety_uncertain);

    s_gpio_levels[echo_pin] = 0;
    s_fake_time_us = 140000;
    ultrasonic_step(&sensor, 140000);
    ultrasonic_edge(echo_pin, 1, 141000);
    ultrasonic_edge(echo_pin, 0, 179000);
    ultrasonic_step(&sensor, 180000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(event.has_echo && event.pulse_us == 38000);
    assert(event.raw_mm == 6517);
    assert(event.quality == ULTRASONIC_QUALITY_NO_RETURN);
    assert(!event.echo_high && !event.safety_uncertain);

    s_fake_time_us = 210000;
    ultrasonic_step(&sensor, 210000);
    ultrasonic_step(&sensor, 256000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(!event.has_echo && !event.echo_high);
    assert(!event.safety_uncertain);

    s_gpio_levels[echo_pin] = 1;
    ultrasonic_restart_session(&sensor, 300000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(!event.has_echo && event.echo_high && event.safety_uncertain);
    ultrasonic_step(&sensor, 310000);
    assert(!sensor.active);
    s_gpio_levels[echo_pin] = 0;
    ultrasonic_step(&sensor, 320000);
    assert(!sensor.blocked_echo_high);
    ultrasonic_step(&sensor, 320001);
    assert(sensor.active);
}

static motor_hal_config_t fake_motor_config(void)
{
    return (motor_hal_config_t) {
        .pwm_max_duty = 1023,
        .enable_level = 1,
    };
}

static void test_motor_driver(void)
{
    motor_driver_t driver;
    const motor_hal_config_t config = fake_motor_config();
    reset_hal();
    assert(motor_driver_preinit_safe(&driver, &config) == MOTOR_RESULT_OK);
    assert(motor_driver_init(&driver) == MOTOR_RESULT_OK);
    reset_hal();

    assert(motor_driver_apply(&driver, (motor_command_t) {500, 0, -500}) ==
           MOTOR_RESULT_OK);
    assert(s_hal_event_count == 5);
    assert(s_hal_events[0].kind == HAL_EVENT_DIRECTION);
    assert(s_hal_events[1].kind == HAL_EVENT_DUTY);
    assert(s_hal_events[2].kind == HAL_EVENT_DIRECTION);
    assert(s_hal_events[3].kind == HAL_EVENT_DUTY);
    assert(s_hal_events[4].kind == HAL_EVENT_ENABLE &&
           s_hal_events[4].value == 1);
    const uint32_t write_count = driver.hardware_write_count;
    reset_hal();
    assert(motor_driver_apply(&driver, (motor_command_t) {500, 0, -500}) ==
           MOTOR_RESULT_OK);
    assert(s_hal_event_count == 0);
    assert(driver.hardware_write_count == write_count);

    reset_hal();
    assert(motor_driver_apply(&driver, (motor_command_t) {-500, 0, -500}) ==
           MOTOR_RESULT_OK);
    assert(s_hal_event_count == 3);
    assert(s_hal_events[0].kind == HAL_EVENT_DUTY &&
           s_hal_events[0].value == 0);
    assert(s_hal_events[1].kind == HAL_EVENT_DIRECTION &&
           s_hal_events[1].value == -1);
    assert(s_hal_events[2].kind == HAL_EVENT_DUTY &&
           s_hal_events[2].value > 0);

    reset_hal();
    assert(motor_driver_apply(&driver, (motor_command_t) {0, 0, 0}) ==
           MOTOR_RESULT_OK);
    assert(s_hal_event_count == 5);
    assert(s_hal_events[0].kind == HAL_EVENT_ENABLE &&
           s_hal_events[0].value == 0);
    assert(!motor_driver_enabled(&driver));

    assert(motor_driver_apply(&driver, (motor_command_t) {200, 0, 0}) ==
           MOTOR_RESULT_OK);
    reset_hal();
    s_fail_kind = HAL_EVENT_DUTY;
    assert(motor_driver_apply(&driver, (motor_command_t) {300, 0, 0}) ==
           MOTOR_RESULT_RECOVERABLE_FAULT);
    assert(driver.fault_latched);
    assert(s_hal_events[s_hal_event_count - 1].kind == HAL_EVENT_ENABLE);
    assert(s_hal_events[s_hal_event_count - 1].value == 0);
    reset_hal();
    assert(motor_driver_safe_reinit(&driver) == MOTOR_RESULT_OK);
    assert(!driver.fault_latched && !motor_driver_enabled(&driver));
}

int main(void)
{
    assert(app_config_validate(&APP_CONFIG));
    app_config_t invalid = APP_CONFIG;
    invalid.obstacle.clear_confirm_count = 0;
    assert(!app_config_validate(&invalid));
    invalid = APP_CONFIG;
    invalid.line.search_max_sweep_ms = invalid.line.search_reverse_ms - 1;
    assert(!app_config_validate(&invalid));
    test_kiwi_kinematics();
    test_camera_line_vision();
    test_line_follow_behavior();
    test_obstacle_supervisor();
    test_automatic_bypass_sequence();
    test_start_button();
    test_motor_driver();
    test_ultrasonic_transactions();
    puts("host firmware tests: PASS");
    return 0;
}
