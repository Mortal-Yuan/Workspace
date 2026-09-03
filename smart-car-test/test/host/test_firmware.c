#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "ball_approach.h"
#include "camera_ball_vision.h"
#include "camera_line_vision.h"
#include "camera_preview.h"
#include "driver/gpio.h"
#include "kiwi_kinematics.h"
#include "line_follow.h"
#include "motor_driver.h"
#include "obstacle_supervisor.h"
#include "start_button.h"
#include "startup_maneuver.h"
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
    TEST_BALL_WIDTH = 80,
    TEST_BALL_HEIGHT = 60,
    TEST_BALL_PIXELS = TEST_BALL_WIDTH * TEST_BALL_HEIGHT,
};

static uint8_t s_test_image[TEST_IMAGE_WIDTH * TEST_IMAGE_HEIGHT * 3];
static camera_line_vision_workspace_t s_camera_line_workspace;
static uint8_t s_ball_image[TEST_BALL_PIXELS * 3];
static uint8_t s_full_camera_image[CAMERA_BALL_VISION_MAX_PIXELS * 3];
static camera_ball_vision_workspace_t s_camera_ball_workspace;

static void fill_ball_image(uint8_t red, uint8_t green, uint8_t blue)
{
    for (size_t index = 0; index < TEST_BALL_PIXELS; ++index) {
        s_ball_image[index * 3] = red;
        s_ball_image[index * 3 + 1] = green;
        s_ball_image[index * 3 + 2] = blue;
    }
}

static void draw_ball_circle(int center_x, int center_y, int radius,
                             uint8_t red, uint8_t green, uint8_t blue)
{
    for (int y = 0; y < TEST_BALL_HEIGHT; ++y) {
        for (int x = 0; x < TEST_BALL_WIDTH; ++x) {
            const int dx = x - center_x;
            const int dy = y - center_y;
            if (dx * dx + dy * dy > radius * radius) continue;
            uint8_t *pixel = &s_ball_image[
                (y * TEST_BALL_WIDTH + x) * 3];
            pixel[0] = red;
            pixel[1] = green;
            pixel[2] = blue;
        }
    }
}

static camera_ball_observation_t analyze_ball_image(void)
{
    return camera_ball_analyze_rgb888(
        s_ball_image, TEST_BALL_WIDTH, TEST_BALL_HEIGHT, false,
        &APP_CONFIG.camera_ball,
        &s_camera_ball_workspace);
}

static void test_camera_ball_vision(void)
{
    assert(APP_CONFIG.camera_ball.red_minimum == 45);
    assert(APP_CONFIG.camera_ball.red_dominance == 8);
    assert(APP_CONFIG.camera_ball.red_ratio_permille == 380);
    assert(APP_CONFIG.camera_ball.strong_red_dominance == 40);
    assert(APP_CONFIG.camera_ball.strong_red_ratio_permille == 420);
    assert(APP_CONFIG.camera_ball.blue_minimum == 70);
    assert(APP_CONFIG.camera_ball.blue_dominance == 16);
    assert(APP_CONFIG.camera_ball.blue_ratio_permille == 390);
    assert(APP_CONFIG.camera_ball.strong_blue_dominance == 24);
    assert(APP_CONFIG.camera_ball.strong_blue_ratio_permille == 390);
    assert(APP_CONFIG.camera_ball.minimum_strong_pixels == 4);
    assert(APP_CONFIG.camera_ball.minimum_strong_ratio_permille == 80);
    assert(APP_CONFIG.camera_ball.minimum_mean_red_dominance == 24);
    assert(APP_CONFIG.camera_ball.minimum_mean_blue_dominance == 55);
    assert(APP_CONFIG.camera_ball.minimum_blue_roundness_permille == 350);
    assert(APP_CONFIG.camera_ball.blue_target_minimum_pixels == 3);
    assert(APP_CONFIG.camera_ball.blue_target_minimum_strong_pixels == 2);
    assert(APP_CONFIG.camera_ball.blue_target_minimum_mean_dominance == 24);
    assert(APP_CONFIG.camera_ball.blue_target_minimum_confidence_permille ==
           720);
    assert(APP_CONFIG.camera_ball.blue_target_minimum_roundness_permille ==
           250);
    assert(APP_CONFIG.camera_ball.minimum_area_permille == 4);
    assert(APP_CONFIG.camera_ball.far_minimum_area_permille == 2);
    assert(APP_CONFIG.camera_ball.far_minimum_confidence_permille == 900);
    assert(APP_CONFIG.camera_ball.highlight_minimum == 160);
    assert(APP_CONFIG.camera_ball.highlight_max_chroma == 48);
    assert(APP_CONFIG.camera_ball.highlight_expand_passes == 2);
    assert(APP_CONFIG.camera_ball.edge_margin_pixels == 2);
    assert(APP_CONFIG.camera_ball.tracking_tolerance_permille == 200);
    assert(APP_CONFIG.camera_ball.confirm_frames == 3);
    assert(APP_CONFIG.camera_ball.far_confirm_frames == 5);

    fill_ball_image(120, 120, 120);
    camera_ball_observation_t ball = analyze_ball_image();
    assert(ball.valid && !ball.candidate);
    assert(ball.probe_red_score == 0);

    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 9, 185, 35, 25);
    ball = analyze_ball_image();
    assert(ball.valid && ball.candidate && !ball.detected);
    assert(ball.color == BALL_COLOR_RED);
    assert(ball.center_x_permille > -30 && ball.center_x_permille < 30);
    assert(ball.center_y_permille > 450 && ball.center_y_permille < 550);
    assert(ball.width_permille > 200 && ball.height_permille > 250);
    assert(ball.fill_permille >= 690);
    assert(ball.roundness_permille > 900);
    assert(ball.mean_red == 185 && ball.mean_green == 35 &&
           ball.mean_blue == 25);
    assert(ball.probe_red_score == 310 && ball.probe_red == 185 &&
           ball.probe_green == 35 && ball.probe_blue == 25);
    assert(ball.strong_pixels == ball.matched_pixels);
    assert(ball.highlight_pixels == 0);
    assert(ball.box_left_permille == 392 &&
           ball.box_top_permille == 356 &&
           ball.box_right_permille == 620 &&
           ball.box_bottom_permille == 661);

    /* A 3x4, high-confidence red target represents the measured 40 cm ball.
     * It is admitted only through the far gate and will require five decoded
     * frames in the camera driver before detected becomes true. */
    fill_ball_image(120, 120, 120);
    for (int y = 28; y <= 31; ++y) {
        for (int x = 39; x <= 41; ++x) {
            uint8_t *pixel = &s_ball_image[
                (y * TEST_BALL_WIDTH + x) * 3];
            pixel[0] = 185;
            pixel[1] = 35;
            pixel[2] = 25;
        }
    }
    ball = analyze_ball_image();
    assert(ball.candidate && ball.far_candidate && !ball.detected);
    assert(ball.area_permille >=
           APP_CONFIG.camera_ball.far_minimum_area_permille);
    assert(ball.area_permille < APP_CONFIG.camera_ball.minimum_area_permille);
    assert(ball.confidence_permille >=
           APP_CONFIG.camera_ball.far_minimum_confidence_permille);

    /* A dim red edge and a white specular center remain one seeded ball.
     * Its center is above the independent line ROI to prove full-frame use. */
    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 20, 9, 125, 90, 70);
    draw_ball_circle(40, 20, 6, 175, 75, 55);
    draw_ball_circle(38, 18, 3, 235, 230, 228);
    ball = analyze_ball_image();
    assert(ball.valid && ball.candidate);
    assert(ball.center_y_permille < APP_CONFIG.camera_line.roi_top_permille);
    assert(ball.strong_pixels >= 4);
    assert(ball.matched_pixels > ball.strong_pixels);
    assert(ball.highlight_pixels > 0);
    assert(ball.fill_permille >= 600 && ball.roundness_permille > 900);

    /* A bright neutral spot cannot seed a ball on its own. */
    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 9, 235, 230, 228);
    ball = analyze_ball_image();
    assert(ball.valid && !ball.candidate && ball.strong_pixels == 0);

    fill_ball_image(120, 120, 120);
    draw_ball_circle(18, 42, 7, 170, 35, 30);
    ball = analyze_ball_image();
    assert(ball.candidate && ball.center_x_permille < -400 &&
           ball.center_y_permille > 600);

    fill_ball_image(120, 120, 120);
    draw_ball_circle(1, 30, 7, 170, 35, 30);
    ball = analyze_ball_image();
    assert(ball.color == BALL_COLOR_RED && !ball.candidate);

    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 9, 25, 190, 30);
    ball = analyze_ball_image();
    assert(ball.valid && !ball.candidate);
    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 9, 25, 45, 190);
    ball = analyze_ball_image();
    assert(ball.valid && ball.candidate && ball.color == BALL_COLOR_BLUE);

    /* Live 80x60 calibration: the centered blue target occupies x=34..45,
     * y=28..32 with mean RGB approximately 126/157/251.  Its 12:5 rectangle
     * uses the blue-specific shape bound while retaining seeded color, area,
     * fill, edge, and confirmation gates. */
    fill_ball_image(120, 120, 120);
    for (int y = 28; y <= 32; ++y) {
        for (int x = 34; x <= 45; ++x) {
            uint8_t *pixel = &s_ball_image[
                (y * TEST_BALL_WIDTH + x) * 3];
            pixel[0] = 126;
            pixel[1] = 157;
            pixel[2] = 251;
        }
    }
    ball = analyze_ball_image();
    assert(ball.valid && ball.candidate && ball.color == BALL_COLOR_BLUE);
    assert(ball.matched_pixels == 60 && ball.strong_pixels == 60);
    assert(ball.box_left_permille == 430 &&
           ball.box_top_permille == 475 &&
           ball.box_right_permille == 570 &&
           ball.box_bottom_permille == 542);
    assert(ball.roundness_permille >= 400 &&
           ball.roundness_permille < 500);

    /* Red and blue must be available as two independent observations from
     * the same decoded frame. */
    fill_ball_image(120, 120, 120);
    draw_ball_circle(23, 39, 7, 185, 35, 25);
    for (int y = 18; y <= 23; ++y) {
        for (int x = 50; x <= 62; ++x) {
            uint8_t *pixel = &s_ball_image[
                (y * TEST_BALL_WIDTH + x) * 3];
            pixel[0] = 126;
            pixel[1] = 157;
            pixel[2] = 251;
        }
    }
    const camera_ball_observation_t red_ball =
        camera_ball_analyze_color_rgb888(
            s_ball_image, TEST_BALL_WIDTH, TEST_BALL_HEIGHT, false,
            BALL_COLOR_RED, &APP_CONFIG.camera_ball,
            &s_camera_ball_workspace);
    const camera_ball_observation_t blue_goal =
        camera_ball_analyze_color_rgb888(
            s_ball_image, TEST_BALL_WIDTH, TEST_BALL_HEIGHT, false,
            BALL_COLOR_BLUE, &APP_CONFIG.camera_ball,
            &s_camera_ball_workspace);
    assert(red_ball.candidate && red_ball.color == BALL_COLOR_RED &&
           red_ball.center_x_permille < -300);
    assert(blue_goal.candidate && blue_goal.color == BALL_COLOR_BLUE &&
           blue_goal.center_x_permille > 300);

    /* Left and right blue targets are selected independently.  Even a
     * three-pixel component touching the image edge remains a far candidate,
     * while the other side is returned at the same time. */
    fill_ball_image(120, 120, 120);
    for (int x = 0; x <= 2; ++x) {
        uint8_t *pixel = &s_ball_image[(10 * TEST_BALL_WIDTH + x) * 3];
        pixel[0] = 40;
        pixel[1] = 80;
        pixel[2] = 230;
    }
    for (int y = 30; y <= 31; ++y) {
        for (int x = 77; x <= 78; ++x) {
            uint8_t *pixel = &s_ball_image[
                (y * TEST_BALL_WIDTH + x) * 3];
            pixel[0] = 40;
            pixel[1] = 80;
            pixel[2] = 230;
        }
    }
    const camera_blue_target_pair_t blue_targets =
        camera_blue_targets_analyze_rgb888(
            s_ball_image, TEST_BALL_WIDTH, TEST_BALL_HEIGHT, false,
            &APP_CONFIG.camera_ball, &s_camera_ball_workspace);
    assert(blue_targets.left_target.candidate &&
           blue_targets.left_target.far_candidate &&
           blue_targets.left_target.matched_pixels == 3 &&
           blue_targets.left_target.center_x_permille < -900);
    assert(blue_targets.right_target.candidate &&
           blue_targets.right_target.far_candidate &&
           blue_targets.right_target.matched_pixels == 4 &&
           blue_targets.right_target.center_x_permille > 900);

    /* The larger pale-blue live background sample has insufficient mean blue
     * dominance and must not become a target. */
    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 9, 206, 182, 255);
    ball = analyze_ball_image();
    assert(ball.valid && !ball.candidate);

    /* A round but weakly red background region must not outrank the ball. */
    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 9, 98, 74, 47);
    ball = analyze_ball_image();
    assert(ball.valid && ball.color == BALL_COLOR_RED && !ball.candidate);

    /* Red noise below the configured area and an elongated red object
     * are both rejected as balls. */
    fill_ball_image(120, 120, 120);
    draw_ball_circle(40, 30, 2, 185, 35, 25);
    ball = analyze_ball_image();
    assert(!ball.candidate);
    fill_ball_image(120, 120, 120);
    for (int y = 28; y <= 31; ++y) {
        for (int x = 15; x <= 64; ++x) {
            uint8_t *pixel = &s_ball_image[
                (y * TEST_BALL_WIDTH + x) * 3];
            pixel[0] = 185;
            pixel[1] = 35;
            pixel[2] = 25;
        }
    }
    ball = analyze_ball_image();
    assert(!ball.candidate);

    /* Exercise the exact 80x60 decoded shape passed by the UVC driver. */
    for (size_t index = 0; index < CAMERA_BALL_VISION_MAX_PIXELS; ++index) {
        s_full_camera_image[index * 3U] = 120;
        s_full_camera_image[index * 3U + 1U] = 120;
        s_full_camera_image[index * 3U + 2U] = 120;
    }
    const int native_center_x = CAMERA_BALL_VISION_MAX_WIDTH / 2;
    const int native_center_y = CAMERA_BALL_VISION_MAX_HEIGHT / 2;
    const int native_radius = 9 *
        (CAMERA_BALL_VISION_MAX_WIDTH / TEST_BALL_WIDTH);
    for (int y = 0; y < CAMERA_BALL_VISION_MAX_HEIGHT; ++y) {
        for (int x = 0; x < CAMERA_BALL_VISION_MAX_WIDTH; ++x) {
            const int dx = x - native_center_x;
            const int dy = y - native_center_y;
            if (dx * dx + dy * dy > native_radius * native_radius) continue;
            uint8_t *pixel = &s_full_camera_image[
                ((size_t)y * CAMERA_BALL_VISION_MAX_WIDTH + (size_t)x) * 3U];
            pixel[0] = 185;
            pixel[1] = 35;
            pixel[2] = 25;
        }
    }
    ball = camera_ball_analyze_rgb888(
        s_full_camera_image, CAMERA_BALL_VISION_MAX_WIDTH,
        CAMERA_BALL_VISION_MAX_HEIGHT, false, &APP_CONFIG.camera_ball,
        &s_camera_ball_workspace);
    assert(ball.valid && ball.candidate);
    assert(ball.center_x_permille > -10 && ball.center_x_permille < 10);
    assert(ball.center_y_permille > 490 && ball.center_y_permille < 510);
    assert(ball.width_permille > 220 && ball.height_permille > 290);
    assert(ball.box_left_permille > 380 && ball.box_left_permille < 400);
    assert(ball.box_right_permille >= 615 && ball.box_right_permille <= 625);
}

static camera_line_snapshot_t ball_control_frame(
    uint32_t sequence, bool candidate, bool detected, int center_x,
    int center_y, int height, int area, int box_bottom)
{
    return (camera_line_snapshot_t) {
        .fresh = true,
        .frame_valid = true,
        .decoded_frames = sequence,
        .ball = {
            .valid = true,
            .candidate = candidate,
            .detected = detected,
            .color = BALL_COLOR_RED,
            .center_x_permille = (int16_t)center_x,
            .center_y_permille = (uint16_t)center_y,
            .height_permille = (uint16_t)height,
            .area_permille = (uint16_t)area,
            .box_bottom_permille = (uint16_t)box_bottom,
        },
        .left_target = {
            .valid = true,
            .candidate = true,
            .detected = true,
            .color = BALL_COLOR_BLUE,
            .center_x_permille = 25,
            .center_y_permille = 500,
            .width_permille = 180,
            .height_permille = 150,
            .area_permille = 20,
            .box_left_permille = 420,
            .box_top_permille = 420,
            .box_right_permille = 600,
            .box_bottom_permille = 570,
        },
    };
}

static void set_blue_goal(camera_line_snapshot_t *camera, bool candidate,
                          bool detected, int center_x, int center_y,
                          int left, int top, int right, int bottom)
{
    camera->left_target.candidate = candidate;
    camera->left_target.detected = detected;
    camera->left_target.color = candidate ? BALL_COLOR_BLUE :
                                           BALL_COLOR_NONE;
    camera->left_target.center_x_permille = (int16_t)center_x;
    camera->left_target.center_y_permille = (uint16_t)center_y;
    camera->left_target.box_left_permille = (uint16_t)left;
    camera->left_target.box_top_permille = (uint16_t)top;
    camera->left_target.box_right_permille = (uint16_t)right;
    camera->left_target.box_bottom_permille = (uint16_t)bottom;
}

static void set_second_blue_goal(camera_line_snapshot_t *camera,
                                 bool candidate, bool detected,
                                 int center_x, int center_y,
                                 int left, int top, int right, int bottom)
{
    camera->right_target = camera->left_target;
    camera->right_target.candidate = candidate;
    camera->right_target.detected = detected;
    camera->right_target.color = candidate ? BALL_COLOR_BLUE :
                                            BALL_COLOR_NONE;
    camera->right_target.center_x_permille = (int16_t)center_x;
    camera->right_target.center_y_permille = (uint16_t)center_y;
    camera->right_target.box_left_permille = (uint16_t)left;
    camera->right_target.box_top_permille = (uint16_t)top;
    camera->right_target.box_right_permille = (uint16_t)right;
    camera->right_target.box_bottom_permille = (uint16_t)bottom;
}

static void test_ball_approach(void)
{
    const ball_approach_config_t *config = &APP_CONFIG.ball_approach;
    assert(config->target_center_x_permille == 25);
    assert(config->route_lateral_speed == 300);
    assert(config->route_pulse_ms == 80);
    assert(config->capture_center_y_permille == 760);
    assert(config->capture_box_bottom_permille == 850);
    assert(config->capture_height_permille == 130);
    assert(config->capture_area_permille == 10);
    assert(config->capture_confirm_frames == 2);
    assert(config->capture_verify_max_frames == 5);
    assert(config->far_forward_speed == 300);
    assert(config->medium_forward_speed == 250);
    assert(config->near_forward_speed == 210);
    assert(config->forward_boost_speed == 360);
    assert(config->search_speed == 360);
    assert(config->search_pulse_ms == 80);
    assert(config->search_settle_ms == 200);
    assert(config->goal_search_speed == 260);
    assert(config->align_pulse_ms == 80);
    assert(config->align_settle_ms == 200);
    assert(config->push_speed == 300);
    assert(config->push_boost_speed == 420);
    assert(config->push_boost_ms == 180);
    assert(config->goal_overlap_margin_permille == 0);
    assert(config->goal_overlap_confirm_frames == 3);
    assert(config->recovery_wait_ms == 2000);

    ball_approach_t controller;
    ball_approach_init(&controller, config, &APP_CONFIG.kinematics);
    int64_t now_us = 0;

    /* With no red target, each sweep is made from fast 80 ms increments with
     * a full stopped observation between increments.  The accumulated motor
     * time still preserves the original left/right search angles. */
    ball_approach_start(&controller, now_us);
    camera_line_snapshot_t camera = ball_control_frame(
        1, false, false, 0, 0, 0, 0, 0);
    ball_approach_decision_t decision = ball_approach_step(
        &controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_LEFT);
    assert(decision.command.a == -360 && decision.command.c == 360);
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_SETTLE);
    assert(controller.search_motion_ms == config->search_pulse_ms);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_LEFT);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.command.a == -360 && decision.command.c == 360);

    /* Exercise the final left pulse without spending ten pulse/settle cycles
     * in this unit test. */
    controller.search_motion_ms = (uint16_t)(
        config->search_left_ms - config->search_pulse_ms);
    controller.state_started_us = now_us;
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_SETTLE);
    assert(controller.search_motion_ms == 0);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_RIGHT);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.command.a == 360 && decision.command.c == -360);

    /* Ball mode always starts by finding red.  A blue component first seen
     * in the camera's positional-right slot is nevertheless the preferred
     * delivery target; a later higher-confidence positional-left component
     * must not replace its continuous track. */
    ball_approach_start(&controller, now_us += 100000);
    camera = ball_control_frame(2, false, false, 0, 0, 0, 0, 0);
    set_blue_goal(&camera, false, false, 0, 0, 0, 0, 0, 0);
    set_second_blue_goal(&camera, true, true, 300, 350,
                         590, 280, 710, 420);
    camera.right_target.stable_frames = 5;
    camera.right_target.confidence_permille = 900;
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_LEFT);
    assert(controller.goal_preference_locked);
    assert(controller.selected_goal.center_x_permille == 300);

    camera = ball_control_frame(3, true, true, 25, 450, 100, 6, 508);
    camera.left_target.stable_frames = 9;
    camera.left_target.confidence_permille = 1000;
    set_second_blue_goal(&camera, true, true, 280, 350,
                         580, 280, 700, 420);
    camera.right_target.stable_frames = 3;
    camera.right_target.confidence_permille = 700;
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_ROUTE_ALIGN);
    assert(controller.selected_goal.center_x_permille == 280);
    assert(motor_command_is_zero(decision.command));

    ball_approach_start(&controller, now_us);

    camera = ball_control_frame(
        10, true, true, 300, 450, 100, 6, 508);
    set_blue_goal(&camera, true, true, -100, 350, 390, 280, 510, 420);
    decision = ball_approach_step(
        &controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_ROUTE_ALIGN);
    assert(motor_command_is_zero(decision.command));

    camera.decoded_frames = 11;
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_ROUTE_SHIFT);
    assert(!motor_command_is_zero(decision.command));

    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->route_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_ROUTE_SETTLE);
    assert(motor_command_is_zero(decision.command));

    camera = ball_control_frame(12, true, true, 300, 450, 100, 6, 508);
    set_blue_goal(&camera, true, true, 300, 350, 590, 280, 710, 420);
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->align_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_ROUTE_ALIGN);
    for (uint32_t sequence = 13; sequence <= 15; ++sequence) {
        camera.decoded_frames = sequence;
        decision = ball_approach_step(
            &controller, &camera, now_us += 70000);
    }
    assert(decision.state == BALL_APPROACH_STATE_ALIGN);

    camera = ball_control_frame(16, true, true, 300, 450, 100, 6, 508);
    set_blue_goal(&camera, true, true, 300, 350, 590, 280, 710, 420);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN_PULSE);
    assert(decision.command.a > 0 && decision.command.b == 0 &&
           decision.command.c < 0);

    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->align_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN_SETTLE);
    assert(motor_command_is_zero(decision.command));

    camera = ball_control_frame(17, true, true, 25, 450, 100, 6, 508);
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->align_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN);
    for (uint32_t sequence = 18; sequence <= 20; ++sequence) {
        camera = ball_control_frame(
            sequence, true, true, 25, 450, 100, 6, 508);
        decision = ball_approach_step(
            &controller, &camera, now_us += 70000);
    }
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);
    assert(motor_command_is_zero(decision.command));

    camera = ball_control_frame(21, true, true, 25, 500, 100, 6, 508);
    decision = ball_approach_step(&controller, &camera, now_us += 120000);
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);
    assert(decision.command.a == -300 && decision.command.b == 0 &&
           decision.command.c == -300);

    /* The calibrated 10 cm sample must still drive; it is not in the clip. */
    camera = ball_control_frame(22, true, true, 25, 651, 150, 13, 729);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);
    assert(decision.command.a == -250 && decision.command.b == 0 &&
           decision.command.c == -250);

    /* The final pre-capture band uses the original near speed. */
    camera = ball_control_frame(23, true, true, 25, 750, 150, 13, 780);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);
    assert(decision.command.a == -210 && decision.command.b == 0 &&
           decision.command.c == -210);

    /* Exact second live in-clip geometry: the old bottom=880 gate rejected
     * its row-51 box bottom (864) even though P10 remains far away at y=651. */
    camera = ball_control_frame(24, true, true, -80, 780, 150, 9, 864);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_CAPTURE_VERIFY);
    assert(controller.capture_frames == 1);
    assert(motor_command_is_zero(decision.command));

    /* Match the live retry that the former 170/18 size gate rejected even
     * though its center and bottom already showed contact. */
    camera = ball_control_frame(25, true, true, -57, 814, 150, 14, 898);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(!decision.done &&
           decision.state == BALL_APPROACH_STATE_PUSH_ALIGN);
    assert(controller.capture_frames == 2);
    assert(motor_command_is_zero(decision.command));

    /* The captured ball stays at the clip while blue-goal samples confirm
     * the push heading. */
    for (uint32_t sequence = 26; sequence <= 28; ++sequence) {
        camera = ball_control_frame(
            sequence, true, true, 25, 820, 190, 20, 915);
        set_blue_goal(&camera, true, true, 25, 600,
                      420, 500, 600, 700);
        decision = ball_approach_step(
            &controller, &camera, now_us += 70000);
    }
    assert(decision.state == BALL_APPROACH_STATE_PUSH);
    assert(motor_command_is_zero(decision.command));

    camera.decoded_frames = 29;
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_PUSH);
    assert(decision.command.a == -420 && decision.command.b == 0 &&
           decision.command.c == -420);

    /* After the 180 ms breakaway pulse, pushing settles to 300. */
    camera.decoded_frames = 30;
    decision = ball_approach_step(&controller, &camera, now_us += 120000);
    assert(decision.state == BALL_APPROACH_STATE_PUSH);
    assert(decision.command.a == -300 && decision.command.b == 0 &&
           decision.command.c == -300);

    /* A red center inside the actual blue box stops immediately. Completion
     * now requires three stationary observations with no exterior margin. */
    camera.decoded_frames = 31;
    set_blue_goal(&camera, true, true, 25, 790,
                  420, 700, 600, 880);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_VERIFY);
    assert(motor_command_is_zero(decision.command));
    camera.decoded_frames = 32;
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(!decision.done &&
           decision.state == BALL_APPROACH_STATE_GOAL_VERIFY);
    assert(controller.goal_frames == 2);
    assert(motor_command_is_zero(decision.command));
    camera.decoded_frames = 33;
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.done && decision.state == BALL_APPROACH_STATE_DONE);
    assert(controller.goal_frames == 3);
    assert(motor_command_is_zero(decision.command));

    /* A center just outside the target box is no longer accepted by the old
     * 60-permille margin. */
    ball_approach_start(&controller, now_us += 100000);
    controller.state = BALL_APPROACH_STATE_PUSH;
    controller.state_started_us = now_us;
    controller.push_started_us = now_us;
    controller.push_motion_started_us = now_us;
    camera = ball_control_frame(34, true, true, 25, 790, 150, 14, 880);
    set_blue_goal(&camera, true, true, 25, 790,
                  520, 700, 700, 880);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_PUSH);
    assert(!decision.done && !motor_command_is_zero(decision.command));

    /* With the ball captured, a right-side goal is aligned through repeated
     * small rightward translations, not an in-place yaw. */
    ball_approach_start(&controller, now_us += 100000);
    controller.state = BALL_APPROACH_STATE_PUSH_ALIGN;
    controller.state_started_us = now_us;
    controller.push_started_us = now_us;
    camera = ball_control_frame(35, true, true, 25, 820, 190, 20, 915);
    set_blue_goal(&camera, true, true, 300, 600,
                  590, 500, 710, 700);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_PUSH_ALIGN_PULSE);
    assert(decision.command.a == 300 && decision.command.b == 460 &&
           decision.command.c == -300);
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->route_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_PUSH_ALIGN_SETTLE);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->align_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_PUSH_ALIGN);
    assert(motor_command_is_zero(decision.command));

    /* One noisy non-contact sample inside the five-frame stationary window
     * does not immediately latch failsafe; a later matching sample completes
     * the required two accumulated confirmations. */
    ball_approach_start(&controller, now_us += 100000);
    controller.state = BALL_APPROACH_STATE_CAPTURE_VERIFY;
    controller.state_started_us = now_us;
    controller.capture_frames = 1;
    controller.capture_samples = 1;
    camera = ball_control_frame(35, true, true, 25, 760, 120, 8, 850);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_CAPTURE_VERIFY);
    assert(controller.capture_frames == 1 &&
           controller.capture_samples == 2);
    assert(motor_command_is_zero(decision.command));
    camera = ball_control_frame(36, true, true, -57, 814, 150, 14, 898);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_PUSH_ALIGN);
    assert(controller.capture_frames == 2 &&
           controller.capture_samples == 3);
    assert(motor_command_is_zero(decision.command));

    /* A sudden left/right reversal during a turn pulse stops first and is
     * never converted directly into an opposite motor command. */
    ball_approach_start(&controller, now_us += 100000);
    controller.state = BALL_APPROACH_STATE_ALIGN;
    controller.state_started_us = now_us;
    controller.route_completed = true;
    camera = ball_control_frame(40, true, true, 300, 450, 100, 6, 508);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN_PULSE &&
           !motor_command_is_zero(decision.command));
    camera = ball_control_frame(41, true, true, -300, 450, 100, 6, 508);
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN_SETTLE);
    assert(motor_command_is_zero(decision.command));

    /* A confirmed red ball no longer waits for a distant blue destination.
     * It aligns and approaches the red ball first; only after capture is
     * confirmed does it alternate left/right target-search sweeps. */
    ball_approach_start(&controller, now_us += 100000);
    camera = ball_control_frame(50, true, true, 25, 450, 100, 6, 508);
    set_blue_goal(&camera, false, false, 0, 0, 0, 0, 0, 0);
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN);
    assert(motor_command_is_zero(decision.command));
    for (uint32_t sequence = 51; sequence <= 53; ++sequence) {
        camera.decoded_frames = sequence;
        decision = ball_approach_step(
            &controller, &camera, now_us += 70000);
    }
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);
    assert(motor_command_is_zero(decision.command));

    camera = ball_control_frame(54, true, true, 25, 500, 100, 6, 508);
    set_blue_goal(&camera, false, false, 0, 0, 0, 0, 0, 0);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);
    assert(!motor_command_is_zero(decision.command));

    /* If left_target becomes confirmed during that forward approach, stop
     * immediately and restore the ordinary simultaneous ball/target route
     * alignment instead of insisting on reaching the clip first. */
    camera = ball_control_frame(55, true, true, 25, 550, 110, 7, 610);
    set_blue_goal(&camera, true, true, -100, 350,
                  390, 280, 510, 420);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_ROUTE_ALIGN);
    assert(motor_command_is_zero(decision.command));

    /* If it disappears again before route alignment completes, red-ball
     * approach resumes and remains eligible to pre-align on a later return. */
    camera.decoded_frames = 56;
    set_blue_goal(&camera, false, false, 0, 0, 0, 0, 0, 0);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_ALIGN);
    assert(!controller.route_completed);
    for (uint32_t sequence = 57; sequence <= 59; ++sequence) {
        camera.decoded_frames = sequence;
        decision = ball_approach_step(
            &controller, &camera, now_us += 70000);
    }
    assert(decision.state == BALL_APPROACH_STATE_APPROACH);

    camera = ball_control_frame(60, true, true, -57, 814, 150, 14, 898);
    set_blue_goal(&camera, false, false, 0, 0, 0, 0, 0, 0);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_CAPTURE_VERIFY);
    assert(motor_command_is_zero(decision.command));

    camera.decoded_frames = 61;
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_SEARCH_LEFT);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_SEARCH_LEFT);
    assert(decision.command.a == -260 && decision.command.c == 260);
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE);
    assert(controller.search_motion_ms == config->search_pulse_ms);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_SEARCH_LEFT);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.command.a == -260 && decision.command.c == 260);

    /* Finish the accumulated left sweep, then reverse only after another
     * full stopped observation interval. */
    controller.search_motion_ms = (uint16_t)(
        config->search_left_ms - config->search_pulse_ms);
    controller.state_started_us = now_us;
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_pulse_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_SEARCH_SETTLE);
    assert(controller.search_motion_ms == 0);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->search_settle_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_GOAL_SEARCH_RIGHT);
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.command.a == 260 && decision.command.c == -260);

    camera.decoded_frames = 62;
    set_blue_goal(&camera, true, true, 25, 600,
                  420, 500, 600, 700);
    decision = ball_approach_step(&controller, &camera, now_us += 70000);
    assert(decision.state == BALL_APPROACH_STATE_PUSH_ALIGN);
    assert(!decision.failsafe && motor_command_is_zero(decision.command));

    /* Stale vision also waits at zero motor output.  The timer may expire,
     * but restart remains blocked until a fresh frame exists. */
    ball_approach_start(&controller, now_us += 100000);
    camera = ball_control_frame(60, false, false, 0, 0, 0, 0, 0);
    camera.fresh = false;
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(!decision.failsafe &&
           decision.state == BALL_APPROACH_STATE_RECOVERY_WAIT);
    assert(controller.last_reason == BALL_APPROACH_REASON_CAMERA_STALE);
    assert(motor_command_is_zero(decision.command));
    decision = ball_approach_step(
        &controller, &camera,
        now_us += config->recovery_wait_ms * 1000LL);
    assert(decision.state == BALL_APPROACH_STATE_RECOVERY_WAIT);
    camera.fresh = true;
    camera.decoded_frames++;
    decision = ball_approach_step(&controller, &camera, now_us += 20000);
    assert(decision.state == BALL_APPROACH_STATE_SEARCH_LEFT);
    assert(!decision.failsafe && motor_command_is_zero(decision.command));
}

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
    assert(APP_CONFIG.camera_line.maximum_black_gray == 120);
    assert(APP_CONFIG.camera_line.hairpin_near_threshold_permille == 160);
    assert(APP_CONFIG.camera_line.hairpin_heading_threshold_permille == 300);
    assert(APP_CONFIG.camera_line.hairpin_heading_gain_permille == 100);
    assert(APP_CONFIG.camera_line.finish_width_permille == 800);
    assert(APP_CONFIG.camera_line.finish_black_permille == 200);
    assert(APP_CONFIG.camera_line.finish_confirm_frames == 2);
    /* The captured hairpin geometry must not cancel into an almost-straight
     * steering request. */
    assert(camera_line_steering_from_geometry(
               250, -160, &APP_CONFIG.camera_line) == 209);
    assert(camera_line_steering_from_geometry(
               -250, 160, &APP_CONFIG.camera_line) == -209);
    assert(camera_line_steering_from_geometry(
               250, 100, &APP_CONFIG.camera_line) == 175);
    uint8_t finish_frames = 0;
    assert(!camera_line_finish_confirmed(
        true, APP_CONFIG.camera_line.finish_confirm_frames, &finish_frames));
    assert(finish_frames == 1);
    assert(camera_line_finish_confirmed(
        true, APP_CONFIG.camera_line.finish_confirm_frames, &finish_frames));
    assert(finish_frames == 2);
    /* Disabling finish recognition during bypass clears a partial or complete
     * candidate sequence.  Re-enabling after line recovery starts fresh. */
    assert(!camera_line_finish_confirmed(
        false, APP_CONFIG.camera_line.finish_confirm_frames, &finish_frames));
    assert(finish_frames == 0);
    assert(!camera_line_finish_confirmed(
        true, APP_CONFIG.camera_line.finish_confirm_frames, &finish_frames));
    assert(camera_line_finish_confirmed(
        true, APP_CONFIG.camera_line.finish_confirm_frames, &finish_frames));
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

    /* A coherent grey shadow used to become Otsu's dark class.  The absolute
     * black ceiling rejects it even though scene contrast remains valid. */
    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 110, 130);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.contrast >= 30);
    assert(analysis.threshold == 120);
    assert(!analysis.line_detected);

    /* Genuinely dark tape remains below both the adaptive and absolute gates. */
    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 110, 90);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected);
    assert(analysis.threshold < APP_CONFIG.camera_line.maximum_black_gray);

    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 110, 25);
    analysis = analyze_test_image();
    assert(analysis.valid && analysis.line_detected);
    /* Black classification is intentionally tighter than the retired
     * Otsu + contrast / 8 rule: 25/225 now yields 25 + 200/16 = 37. */
    assert(analysis.threshold == 37);
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

    /* The real endpoint is a thin T: a normal vertical stem connected to a
     * near-full-width horizontal strip.  Its area is deliberately below the
     * retired 350-permille broad-patch gate. */
    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 105, 25);
    draw_view_rectangle(40, 99, 119, 105, 25);
    analysis = analyze_test_image();
    assert(analysis.line_detected && analysis.finish_detected);
    assert(analysis.width_permille == 1000);
    assert(analysis.component_area_permille >= 200 &&
           analysis.component_area_permille < 350);

    /* A one-row horizontal artifact connected to the line is too small to
     * qualify even though its instantaneous width spans the ROI. */
    fill_test_image(225);
    draw_view_rectangle(74, 72, 85, 110, 25);
    draw_view_rectangle(40, 90, 119, 90, 25);
    analysis = analyze_test_image();
    assert(analysis.line_detected && !analysis.finish_detected);

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

    /* Exercise the exact 80x60 decoded shape passed by the UVC driver. */
    for (size_t index = 0; index < CAMERA_BALL_VISION_MAX_PIXELS; ++index) {
        s_full_camera_image[index * 3U] = 225;
        s_full_camera_image[index * 3U + 1U] = 225;
        s_full_camera_image[index * 3U + 2U] = 225;
    }
    const int native_line_left = CAMERA_BALL_VISION_MAX_WIDTH / 2 - 10;
    const int native_line_right = CAMERA_BALL_VISION_MAX_WIDTH / 2 + 9;
    const int native_line_top =
        CAMERA_BALL_VISION_MAX_HEIGHT * 625 / 1000;
    const int native_line_bottom =
        CAMERA_BALL_VISION_MAX_HEIGHT * 925 / 1000;
    for (int y = native_line_top; y <= native_line_bottom; ++y) {
        for (int x = native_line_left; x <= native_line_right; ++x) {
            uint8_t *pixel = &s_full_camera_image[
                ((size_t)y * CAMERA_BALL_VISION_MAX_WIDTH + (size_t)x) * 3U];
            pixel[0] = 25;
            pixel[1] = 25;
            pixel[2] = 25;
        }
    }
    camera_line_config_t native_config = APP_CONFIG.camera_line;
    native_config.center_offset_permille = 0;
    analysis = camera_line_analyze_lower_half_rgb888(
        s_full_camera_image, CAMERA_BALL_VISION_MAX_WIDTH,
        CAMERA_BALL_VISION_MAX_HEIGHT, false, &native_config,
        &s_camera_line_workspace);
    assert(analysis.valid && analysis.line_detected &&
           analysis.connected_component_count == 1 &&
           analysis.center_permille > -20 &&
           analysis.center_permille < 20);
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
    return obstacle_supervisor_step(supervisor, event, s_fake_time_us);
}

static obstacle_decision_t obstacle_step_at(
    obstacle_supervisor_t *supervisor, const ultrasonic_event_t *event,
    line_sensor_sample_t sensors, int64_t now_us)
{
    (void)sensors;
    return obstacle_supervisor_step(supervisor, event, now_us);
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
        (body_motion_command_t) {.clockwise = -420}, &ideal);
    assert(command.a == -420 && command.b == 420 && command.c == 420);

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
    assert(APP_CONFIG.kinematics.lateral_yaw_compensation_permille == 550);
    assert(command.a == -300 && command.b == -589 && command.c == 300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = 500}, &APP_CONFIG.kinematics);
    assert(command.a == -300 && command.b == -775 && command.c == 300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -380}, &APP_CONFIG.kinematics);
    assert(APP_CONFIG.kinematics.right_lateral_yaw_compensation_permille ==
           500);
    assert(command.a == 300 && command.b == 570 && command.c == -300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -300}, &APP_CONFIG.kinematics);
    assert(command.a == 300 && command.b == 460 && command.c == -300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -300, .clockwise = -60},
        &APP_CONFIG.kinematics);
    assert(command.a == 300 && command.b == 510 && command.c == -300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -340, .clockwise = -30},
        &APP_CONFIG.kinematics);
    assert(command.a == 300 && command.b == 540 && command.c == -300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -500}, &APP_CONFIG.kinematics);
    assert(command.a == 300 && command.b == 750 && command.c == -300);
}

static void test_startup_maneuver(void)
{
    const startup_maneuver_config_t *config =
        &APP_CONFIG.startup_maneuver;
    assert(config->forward_speed == 400);
    assert(config->forward_start_speed == 500);
    assert(config->forward_boost_ms == 150);
    assert(config->forward_ms == 1074);
    assert(config->settle_ms == 150);
    assert(config->right_turn_speed == 420);
    assert(config->right_turn_ms == 200);

    startup_maneuver_t maneuver;
    startup_maneuver_init(&maneuver, config);
    assert(maneuver.phase == STARTUP_MANEUVER_WAIT_FOR_CLEAR);
    assert(!startup_maneuver_is_complete(&maneuver));

    const int64_t started_us = 1000000;
    startup_maneuver_decision_t decision =
        startup_maneuver_step(&maneuver, started_us);
    assert(decision.transition ==
           STARTUP_MANEUVER_TRANSITION_TO_FORWARD);
    assert(decision.motion.forward == 500 &&
           decision.motion.clockwise == 0 && !decision.complete);
    motor_command_t wheels = kiwi_inverse_kinematics(
        decision.motion, &APP_CONFIG.kinematics);
    assert(wheels.a == -500 && wheels.b == 0 && wheels.c == -500);

    decision = startup_maneuver_step(
        &maneuver, started_us + 150000 - 1);
    assert(decision.motion.forward == 500);
    decision = startup_maneuver_step(
        &maneuver, started_us + 150000);
    assert(decision.motion.forward == 400);
    decision = startup_maneuver_step(
        &maneuver, started_us + 1074000 - 1);
    assert(decision.motion.forward == 400);

    decision = startup_maneuver_step(
        &maneuver, started_us + 1074000);
    assert(decision.transition ==
           STARTUP_MANEUVER_TRANSITION_TO_SETTLE_AFTER_FORWARD);
    assert(body_motion_zero().forward == decision.motion.forward &&
           decision.motion.clockwise == 0);
    decision = startup_maneuver_step(
        &maneuver, started_us + 1074000 + 150000 - 1);
    assert(decision.motion.forward == 0 &&
           decision.motion.clockwise == 0);

    decision = startup_maneuver_step(
        &maneuver, started_us + 1074000 + 150000);
    assert(decision.transition ==
           STARTUP_MANEUVER_TRANSITION_TO_TURN_RIGHT);
    assert(decision.motion.forward == 0 &&
           decision.motion.clockwise == 420);
    wheels = kiwi_inverse_kinematics(
        decision.motion, &APP_CONFIG.kinematics);
    assert(wheels.a == 420 && wheels.b == -420 && wheels.c == -420);
    decision = startup_maneuver_step(
        &maneuver, started_us + 1074000 + 150000 + 200000 - 1);
    assert(decision.motion.clockwise == 420);

    decision = startup_maneuver_step(
        &maneuver, started_us + 1074000 + 150000 + 200000);
    assert(decision.transition ==
           STARTUP_MANEUVER_TRANSITION_TO_SETTLE_AFTER_TURN);
    assert(decision.motion.clockwise == 0);
    decision = startup_maneuver_step(
        &maneuver,
        started_us + 1074000 + 150000 + 200000 + 150000);
    assert(decision.transition ==
           STARTUP_MANEUVER_TRANSITION_TO_COMPLETE);
    assert(decision.complete && startup_maneuver_is_complete(&maneuver));
    assert(decision.motion.forward == 0 &&
           decision.motion.clockwise == 0);

    startup_maneuver_reset(&maneuver);
    assert(maneuver.phase == STARTUP_MANEUVER_WAIT_FOR_CLEAR);
    assert(!startup_maneuver_is_complete(&maneuver));
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
    assert(config.stop_mm == 80);
    assert(config.no_echo_limit == 3);
    assert(APP_CONFIG.ultrasonic.timeout_us == 45000);
    assert(APP_CONFIG.ultrasonic.period_ms == 70);
    assert(config.lateral_speed == 380);
    assert(config.lateral_start_speed == 500);
    assert(config.left_strafe_ms == 1030);
    assert(config.forward_drive_ms == 1074);
    assert(config.right_lateral_start_speed == 300);
    assert(config.right_lateral_start_clockwise == -60);
    assert(config.right_lateral_ramp_ms == 400);
    assert(config.right_strafe_ms == 1097);
    assert(config.post_bypass_forward_ms == 525);
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

    event = ultrasonic(7, true, 81, 81, ULTRASONIC_QUALITY_VALID,
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
    event = ultrasonic(12, true, 80, 300, ULTRASONIC_QUALITY_OUTLIER,
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

    event = ultrasonic(4, true, 80, 300,
                       ULTRASONIC_QUALITY_OUTLIER, false, false);
    decision = obstacle_step_at(&supervisor, &event, white, 200000);
    assert(supervisor.state == OBSTACLE_STATE_BRAKE);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.line_action == LINE_ACTION_SUSPEND);

    decision = obstacle_step_at(&supervisor, NULL, white,
                                200000 + config.brake_ms * 1000LL);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_LEFT_DISTANCE);
    assert(decision.transition ==
           OBSTACLE_TRANSITION_TO_STRAFE_LEFT_DISTANCE);
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
    assert(decision.override_motion.left ==
           -config.right_lateral_start_speed);
    assert(decision.override_motion.clockwise ==
           config.right_lateral_start_clockwise);

    /* Right strafe accelerates linearly from 300 to 380 over 400 ms.  A small
     * counter-clockwise launch correction fades from -60 to zero during the
     * same interval so the dead-zone-clamped A/C wheels do not yaw the body. */
    const int64_t right_started_us = supervisor.phase_started_us;
    obstacle_supervisor_t ramp_supervisor = supervisor;
    decision = obstacle_step_at(
        &ramp_supervisor, NULL, white,
        right_started_us + config.right_lateral_ramp_ms * 500LL);
    assert(ramp_supervisor.state == OBSTACLE_STATE_STRAFE_RIGHT_DISTANCE);
    assert(decision.override_motion.left == -340);
    assert(decision.override_motion.clockwise == -30);
    decision = obstacle_step_at(
        &ramp_supervisor, NULL, white,
        right_started_us + config.right_lateral_ramp_ms * 1000LL);
    assert(ramp_supervisor.state == OBSTACLE_STATE_STRAFE_RIGHT_DISTANCE);
    assert(decision.override_motion.left == -config.lateral_speed);
    assert(decision.override_motion.clockwise == 0);

    /* Line input cannot end right strafe early after avoidance has started. */
    now_us = right_started_us + config.right_strafe_ms * 500LL;
    decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_RIGHT_DISTANCE);
    assert(decision.policy == MOTION_POLICY_OVERRIDE);
    assert(decision.override_motion.left == -config.lateral_speed);

    now_us = right_started_us + config.right_strafe_ms * 1000LL - 1;
    decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_RIGHT_DISTANCE);
    assert(decision.policy == MOTION_POLICY_OVERRIDE);
    now_us++;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_POST_BYPASS_FORWARD);
    assert(decision.transition ==
           OBSTACLE_TRANSITION_TO_POST_BYPASS_FORWARD);
    assert(decision.reason == OBSTACLE_REASON_SEGMENT_COMPLETE);
    assert(decision.policy == MOTION_POLICY_OVERRIDE);
    assert(decision.override_motion.forward == config.forward_start_speed);
    assert(decision.line_action == LINE_ACTION_KEEP);

    /* All line shapes are ignored while the fixed final-forward segment runs. */
    const int64_t post_forward_started_us = supervisor.phase_started_us;
    now_us = post_forward_started_us +
        config.post_bypass_forward_ms * 1000LL - 1;
    decision = obstacle_step_at(&supervisor, NULL, all_black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_POST_BYPASS_FORWARD);
    assert(decision.policy == MOTION_POLICY_OVERRIDE);
    assert(decision.override_motion.forward == config.forward_speed);
    assert(decision.transition == OBSTACLE_TRANSITION_NONE);

    now_us++;
    decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_FINISHED);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_FINISHED);
    assert(decision.reason == OBSTACLE_REASON_POST_BYPASS_COMPLETE);
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

static void test_camera_preview_packet(void)
{
    uint8_t pixels[CAMERA_PREVIEW_PIXEL_COUNT * 3U];
    memset(pixels, 255, sizeof(pixels));
    const size_t black_index = 45U * CAMERA_PREVIEW_WIDTH + 22U;
    pixels[black_index * 3U] = 0;
    pixels[black_index * 3U + 1U] = 0;
    pixels[black_index * 3U + 2U] = 0;

    const camera_line_analysis_t analysis = {
        .valid = true,
        .line_detected = true,
        .center_permille = 0,
        .far_center_permille = 200,
        .center_y_permille = 800,
        .far_center_y_permille = 200,
        .steering_permille = 100,
        .threshold = 100,
        .contrast = 120,
    };
    const camera_ball_observation_t ball = {
        .valid = true,
        .candidate = true,
        .detected = true,
        .color = BALL_COLOR_RED,
        .center_x_permille = -125,
        .center_y_permille = 250,
        .box_left_permille = 380,
        .box_top_permille = 169,
        .box_right_permille = 506,
        .box_bottom_permille = 339,
    };
    const camera_ball_observation_t left_target = {
        .valid = true,
        .candidate = true,
        .detected = true,
        .color = BALL_COLOR_BLUE,
        .center_x_permille = -500,
        .center_y_permille = 250,
        .box_left_permille = 190,
        .box_top_permille = 169,
        .box_right_permille = 310,
        .box_bottom_permille = 339,
    };
    const camera_ball_observation_t right_target = {
        .valid = true,
        .candidate = true,
        .detected = true,
        .color = BALL_COLOR_BLUE,
        .center_x_permille = 500,
        .center_y_permille = 250,
        .box_left_permille = 690,
        .box_top_permille = 169,
        .box_right_permille = 810,
        .box_bottom_permille = 339,
    };
    camera_preview_packet_t packet;
    assert(camera_preview_build_rgb332(
        &packet, pixels, CAMERA_PREVIEW_WIDTH, CAMERA_PREVIEW_HEIGHT,
        &APP_CONFIG.camera_line, &analysis, &ball, &left_target,
        &right_target, 1234, 5678));
    assert(sizeof(packet) == 4836);
    assert(packet.magic[0] == 0xa5 && packet.magic[1] == 0x5a);
    assert(packet.version == CAMERA_PREVIEW_VERSION);
    assert(packet.width == CAMERA_PREVIEW_WIDTH);
    assert(packet.height == CAMERA_PREVIEW_HEIGHT);
    assert(packet.payload_size == CAMERA_PREVIEW_PIXEL_COUNT);
    assert(packet.sequence == 1234 && packet.timestamp_ms == 5678);
    assert((packet.flags & CAMERA_PREVIEW_FLAG_LINE_DETECTED) != 0);
    assert((packet.flags & CAMERA_PREVIEW_FLAG_BALL_CANDIDATE) != 0);
    assert((packet.flags & CAMERA_PREVIEW_FLAG_BALL_DETECTED) != 0);
    assert(packet.pixels[0] == 0xff); /* white RGB332 outside ROI */
    assert(packet.pixels[black_index] == 0xe0); /* marked black pixel */
    assert(packet.pixels[36U * CAMERA_PREVIEW_WIDTH + 30U] == 0xfc);
    assert(packet.pixels[50U * CAMERA_PREVIEW_WIDTH + 42U] == 0x1c);
    assert(packet.pixels[39U * CAMERA_PREVIEW_WIDTH + 46U] == 0x03);
    /* Confirmed-ball magenta box is drawn above/outside the line ROI. */
    assert(packet.pixels[10U * CAMERA_PREVIEW_WIDTH + 30U] == 0xe3);
    assert(packet.pixels[20U * CAMERA_PREVIEW_WIDTH + 40U] == 0xe3);
    assert(packet.pixels[15U * CAMERA_PREVIEW_WIDTH + 35U] == 0xe3);
    assert(packet.pixels[10U * CAMERA_PREVIEW_WIDTH + 15U] == 0x1f);
    assert(packet.pixels[15U * CAMERA_PREVIEW_WIDTH + 20U] == 0x1f);
    assert(packet.pixels[10U * CAMERA_PREVIEW_WIDTH + 55U] == 0x1d);
    assert(packet.pixels[15U * CAMERA_PREVIEW_WIDTH + 60U] == 0x1d);
    assert(packet.payload_crc32 == camera_preview_crc32(
        packet.pixels, packet.payload_size));

    /* A larger recognition frame must be downsampled into the unchanged
     * low-bandwidth 80x60 wire packet. */
    camera_preview_packet_t scaled_packet;
    assert(camera_preview_build_rgb332(
        &scaled_packet, s_test_image, TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT,
        &APP_CONFIG.camera_line, &analysis, &ball, &left_target,
        &right_target, 1235, 5679));
    assert(scaled_packet.width == CAMERA_PREVIEW_WIDTH &&
           scaled_packet.height == CAMERA_PREVIEW_HEIGHT &&
           scaled_packet.sequence == 1235);
    assert(scaled_packet.payload_crc32 == camera_preview_crc32(
        scaled_packet.pixels, scaled_packet.payload_size));
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
    invalid = APP_CONFIG;
    invalid.startup_maneuver.right_turn_ms = 0;
    assert(!app_config_validate(&invalid));
    invalid = APP_CONFIG;
    invalid.ball_approach.capture_box_bottom_permille = 1001;
    assert(!app_config_validate(&invalid));
    test_kiwi_kinematics();
    test_startup_maneuver();
    test_camera_line_vision();
    test_camera_preview_packet();
    test_camera_ball_vision();
    test_ball_approach();
    test_line_follow_behavior();
    test_obstacle_supervisor();
    test_automatic_bypass_sequence();
    test_start_button();
    test_motor_driver();
    test_ultrasonic_transactions();
    puts("host firmware tests: PASS");
    return 0;
}
