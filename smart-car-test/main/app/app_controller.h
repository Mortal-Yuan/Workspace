#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_events.h"
#include "diagnostics.h"
#include "encoder.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "obstacle_supervisor.h"
#include "start_button.h"
#include "ultrasonic.h"

#define APP_CONTROL_TASK_STACK 6144

typedef enum {
    SELF_TEST_NONE,
    SELF_TEST_DIRECT_TURN,
    SELF_TEST_MOTOR_SEQUENCE,
    SELF_TEST_STRAFE_LEFT,
    SELF_TEST_STRAFE_RIGHT,
    SELF_TEST_FORWARD_CALIBRATION,
    SELF_TEST_MOTOR_B_NEGATIVE,
    SELF_TEST_MOTOR_B_POSITIVE,
} self_test_kind_t;

typedef struct {
    self_test_kind_t kind;
    uint8_t phase;
    char command;
    int64_t deadline_us;
} self_test_t;

typedef struct {
    const app_config_t *config;
    app_mode_t mode;
    motor_driver_t motor;
    line_sensor_t line_sensor;
    encoder_context_t encoder;
    ultrasonic_t ultrasonic;
    start_button_t button;
    line_follow_t line_follow;
    obstacle_supervisor_t obstacle;
    diagnostics_t diagnostics;
    line_sensor_sample_t latest_line;
    fault_record_t faults[FAULT_SOURCE_COUNT];
    uint32_t fault_bitmap;
    uint32_t degraded_bitmap;
    motor_command_t manual_command;
    self_test_t self_test;
    int speed;
    bool line_monitor;
    bool button_ready;
    bool diagnostics_ready;
    bool initialized;
    uint32_t control_overruns;
    uint32_t missed_periods;
    uint32_t max_execution_us;
    int64_t next_snapshot_us;
    StaticTask_t task_buffer;
    StackType_t task_stack[APP_CONTROL_TASK_STACK];
    TaskHandle_t task;
} app_controller_t;

bool app_controller_init(app_controller_t *controller,
                         const app_config_t *config);
bool app_controller_start(app_controller_t *controller);
