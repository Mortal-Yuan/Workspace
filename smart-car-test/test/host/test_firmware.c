#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
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
    assert(command.a == -300 && command.b == -456 && command.c == 300);
    command = kiwi_inverse_kinematics(
        (body_motion_command_t) {.left = -380}, &APP_CONFIG.kinematics);
    assert(command.a == 300 && command.b == 460 && command.c == -300);
}

static void test_line_follow_behavior(void)
{
    line_follow_t controller;
    line_follow_init(&controller, &APP_CONFIG.line);
    line_follow_reset_for_start(&controller, line(false, true, true, false));

    motor_command_t command = line_follow_step(
        &controller, line(false, true, true, false), 450, 0);
    assert(command.a == -500 && command.b == 0 && command.c == -500);
    command = line_follow_step(
        &controller, line(false, true, true, false), 450, 160000);
    assert(command.a == -420 && command.c == -420);

    for (int index = 0; index < 3; ++index) {
        command = line_follow_step(
            &controller, line(false, false, false, true), 450,
            200000 + index * 20000);
    }
    assert(controller.locked_direction == 1);
    command = line_follow_step(&controller, line(false, false, false, false),
                               450, 300000);
    assert(command.a == 500 && command.b == 0 && command.c == -360);
    command = line_follow_step(&controller, line(false, false, false, false),
                               450, 460000);
    assert(command.a == 360 && command.b == 0 && command.c == -360);

    line_follow_reset_for_start(&controller, line(true, false, false, false));
    (void)line_follow_step(&controller, line(true, false, false, false),
                           450, 0);
    command = line_follow_step(&controller, line(true, false, false, false),
                               450, 160000);
    assert(command.a == -450 && command.c == -220);
}

static void test_obstacle_supervisor(void)
{
    obstacle_config_t config = APP_CONFIG.obstacle;
    config.bypass_enabled = false;
    obstacle_supervisor_t supervisor;
    obstacle_supervisor_init(&supervisor, &config);
    ultrasonic_event_t event = ultrasonic(
        1, true, 300, 300, ULTRASONIC_QUALITY_VALID, false, false);
    obstacle_decision_t decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK && decision.clear_count == 1);
    event.seq = 2;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK && decision.clear_count == 2);
    event.seq = 3;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_CLEAR);

    event = ultrasonic(4, true, 200, 300, ULTRASONIC_QUALITY_OUTLIER,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.transition == OBSTACLE_TRANSITION_TO_WAIT_CLEAR);
    assert(decision.line_action == LINE_ACTION_SUSPEND);

    event = ultrasonic(5, false, -1, 300, ULTRASONIC_QUALITY_OUTLIER,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.clear_count == 1);
    event.seq = 6;
    decision = obstacle_step(&supervisor, &event);
    assert(decision.clear_count == 2);
    event = ultrasonic(7, true, 100, 100, ULTRASONIC_QUALITY_VALID,
                       false, false);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.clear_count == 0);

    for (uint32_t seq = 8; seq <= 10; ++seq) {
        event = ultrasonic(seq, false, -1, -1, ULTRASONIC_QUALITY_LOST,
                           false, false);
        decision = obstacle_step(&supervisor, &event);
    }
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.reason == OBSTACLE_REASON_THREE_CLEAR);
    assert(decision.line_action == LINE_ACTION_RESUME);

    event = ultrasonic(11, true, 350, 350, ULTRASONIC_QUALITY_INVALID,
                       false, true);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.reason == OBSTACLE_REASON_UNCERTAIN);
    decision = obstacle_step(&supervisor, &event);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.clear_count == 0);
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

    event = ultrasonic(4, true, 150, 300,
                       ULTRASONIC_QUALITY_OUTLIER, false, false);
    decision = obstacle_step_at(&supervisor, &event, white, 200000);
    assert(supervisor.state == OBSTACLE_STATE_BRAKE);
    assert(decision.policy == MOTION_POLICY_BLOCK);
    assert(decision.line_action == LINE_ACTION_SUSPEND);

    decision = obstacle_step_at(&supervisor, NULL, white,
                                200000 + config.brake_ms * 1000LL);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_LEFT_EDGE);
    assert(decision.policy == MOTION_POLICY_OVERRIDE);
    assert(decision.override_motion.left == config.lateral_start_speed);

    int64_t now_us = supervisor.phase_started_us +
                     config.lateral_edge_min_ms * 1000LL;
    for (uint32_t seq = 5; seq < 5U +
         (uint32_t)config.edge_clear_confirm_count; ++seq) {
        event = ultrasonic(seq, false, -1, -1,
                           ULTRASONIC_QUALITY_LOST, false, false);
        now_us += 60000;
        decision = obstacle_step_at(&supervisor, &event, white, now_us);
    }
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_LEFT_CLEARANCE);
    assert(decision.override_motion.left == config.lateral_speed);

    now_us = supervisor.phase_started_us +
             config.lateral_clearance_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_SETTLE_FORWARD);
    assert(decision.policy == MOTION_POLICY_BLOCK);

    now_us += config.brake_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_FORWARD_BYPASS);
    assert(decision.override_motion.forward == config.forward_start_speed);

    now_us += config.forward_bypass_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_SETTLE_RIGHT);
    assert(decision.policy == MOTION_POLICY_BLOCK);

    now_us += config.brake_ms * 1000LL;
    decision = obstacle_step_at(&supervisor, NULL, white, now_us);
    assert(supervisor.state == OBSTACLE_STATE_STRAFE_RIGHT_LINE);
    assert(decision.override_motion.left == -config.lateral_start_speed);

    now_us += 20000;
    decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    assert(supervisor.state == OBSTACLE_STATE_LINE_CONFIRM);
    assert(decision.policy == MOTION_POLICY_BLOCK);

    for (int count = 1; count < config.line_confirm_count; ++count) {
        now_us += 20000;
        decision = obstacle_step_at(&supervisor, NULL, black, now_us);
    }
    assert(supervisor.state == OBSTACLE_STATE_CLEAR);
    assert(decision.policy == MOTION_POLICY_LINE_FOLLOW);
    assert(decision.line_action == LINE_ACTION_RESUME);
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
    ultrasonic_step(&sensor, 31000);
    ultrasonic_event_t event;
    assert(ultrasonic_take_event(&sensor, &event));
    assert(event.has_echo && event.raw_mm == 100);
    assert(event.quality == ULTRASONIC_QUALITY_VALID);

    s_fake_time_us = 60000;
    ultrasonic_step(&sensor, 60000);
    ultrasonic_edge(echo_pin, 1, 61000);
    ultrasonic_edge(echo_pin, 0, 62750);
    ultrasonic_edge(echo_pin, 1, 62800);
    ultrasonic_step(&sensor, 90000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(event.has_echo && event.pulse_us == 1750);
    assert(event.quality == ULTRASONIC_QUALITY_INVALID);
    assert(event.safety_uncertain);

    s_gpio_levels[echo_pin] = 0;
    s_fake_time_us = 120000;
    ultrasonic_step(&sensor, 120000);
    ultrasonic_step(&sensor, 151000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(!event.has_echo && !event.echo_high);
    assert(!event.safety_uncertain);

    s_gpio_levels[echo_pin] = 1;
    ultrasonic_restart_session(&sensor, 200000);
    assert(ultrasonic_take_event(&sensor, &event));
    assert(!event.has_echo && event.echo_high && event.safety_uncertain);
    ultrasonic_step(&sensor, 210000);
    assert(!sensor.active);
    s_gpio_levels[echo_pin] = 0;
    ultrasonic_step(&sensor, 220000);
    assert(!sensor.blocked_echo_high);
    ultrasonic_step(&sensor, 220001);
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
    test_kiwi_kinematics();
    test_line_follow_behavior();
    test_obstacle_supervisor();
    test_automatic_bypass_sequence();
    test_start_button();
    test_motor_driver();
    test_ultrasonic_transactions();
    puts("host firmware tests: PASS");
    return 0;
}
