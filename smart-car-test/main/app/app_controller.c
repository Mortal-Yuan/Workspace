#include "app_controller.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "esp_system.h"
#include "esp_timer.h"
#include "kiwi_kinematics.h"
#include "platform_init.h"
#include "board_pins.h"

enum {
    DEGRADED_ENCODER = 1U << 0,
    DEGRADED_DIAGNOSTICS = 1U << 1,
    DEGRADED_DISPLAY = 1U << 2,
    STRAFE_CALIBRATION_DURATION_US = 1000000,
};

typedef struct {
    bool stop;
    bool force_auto;
    bool boot;
    char motion;
    char self_test;
    bool toggle_monitor;
    int speed_delta;
    bool clear_encoder;
    bool help;
    bool enable_display;
} command_intents_t;

static void enable_status_display(app_controller_t *controller)
{
    if (!controller->display_ready ||
        status_display_is_requested(&controller->display)) {
        return;
    }
    status_display_enable(&controller->display);
}

static void publish_event(app_controller_t *controller,
                          diagnostic_event_kind_t kind,
                          int32_t code, int32_t value,
                          const char *text, int64_t now_us)
{
    if (!controller->diagnostics_ready) return;
    diagnostic_event_t event = {
        .kind = kind,
        .time_us = now_us,
        .code = code,
        .value = value,
    };
    if (text != NULL) {
        snprintf(event.text, sizeof(event.text), "%s", text);
    }
    diagnostics_publish_critical(&controller->diagnostics, &event);
}

static void set_fault(app_controller_t *controller, fault_source_t source,
                      int32_t code, fault_recoverability_t recoverability)
{
    controller->faults[source] = (fault_record_t) {
        .source = source,
        .code = code,
        .recoverability = recoverability,
        .active = true,
    };
    controller->fault_bitmap |= 1U << source;
    controller->mode = APP_MODE_FAULT;
    if (controller->diagnostics_ready) {
        diagnostics_publish_fault(&controller->diagnostics,
                                  controller->faults[source]);
    }
    if (controller->display_ready) {
        enable_status_display(controller);
        const status_display_snapshot_t display_snapshot = {
            .mode = APP_MODE_FAULT,
            .line_pattern = line_sensor_pattern(controller->latest_line),
            .camera_streaming = controller->latest_camera.streaming,
            .camera_frame_valid = controller->latest_camera.frame_valid,
            .camera_fresh = controller->latest_camera.fresh,
            .camera_center_permille =
                controller->latest_camera.center_permille,
            .ultrasonic_mm = -1,
        };
        status_display_publish(&controller->display, &display_snapshot);
    }
}

static void clear_fault(app_controller_t *controller, fault_source_t source)
{
    controller->faults[source].active = false;
    controller->fault_bitmap &= ~(1U << source);
}

static motor_hal_config_t motor_config(void)
{
    return (motor_hal_config_t) {
        .enable_pin = BOARD_PIN_MOTOR_ENABLE,
        .pwm_pin = {BOARD_PIN_MOTOR_A_PWM, BOARD_PIN_MOTOR_B_PWM,
                    BOARD_PIN_MOTOR_C_PWM},
        .in1_pin = {BOARD_PIN_MOTOR_A_IN1, BOARD_PIN_MOTOR_B_IN1,
                    BOARD_PIN_MOTOR_C_IN1},
        .in2_pin = {BOARD_PIN_MOTOR_A_IN2, BOARD_PIN_MOTOR_B_IN2,
                    BOARD_PIN_MOTOR_C_IN2},
        .pwm_channel = {BOARD_MOTOR_A_CHANNEL, BOARD_MOTOR_B_CHANNEL,
                        BOARD_MOTOR_C_CHANNEL},
        .enable_level = 1,
        .pwm_max_duty = 1023,
        .pwm_frequency_hz = 20000,
    };
}

static void start_autonomy(app_controller_t *controller, int64_t now_us,
                           bool force_reset)
{
    if (controller->mode == APP_MODE_AUTONOMOUS && !force_reset) {
        return;
    }
    if (controller->fault_bitmap != 0) {
        if (controller->faults[FAULT_SOURCE_MOTOR].active &&
            controller->faults[FAULT_SOURCE_MOTOR].recoverability ==
                FAULT_RECOVERABLE_EXPLICIT_REINIT &&
            motor_driver_safe_reinit(&controller->motor) == MOTOR_RESULT_OK) {
            clear_fault(controller, FAULT_SOURCE_MOTOR);
        }
        if (controller->fault_bitmap != 0) {
            controller->mode = APP_MODE_FAULT;
            return;
        }
    }
    controller->manual_command = motor_command_zero();
    controller->self_test = (self_test_t) {0};
    if (!controller->latest_camera.fresh) {
        publish_event(controller, DIAGNOSTIC_EVENT_INFO,
                      ESP_ERR_TIMEOUT, 0, "camera_not_ready", now_us);
        return;
    }
    line_follow_reset_for_start(&controller->line_follow,
                                controller->latest_line);
    obstacle_supervisor_reset(&controller->obstacle);
    ultrasonic_restart_session(&controller->ultrasonic, now_us);
    controller->mode = APP_MODE_AUTONOMOUS;
    publish_event(controller, DIAGNOSTIC_EVENT_MODE,
                  APP_MODE_AUTONOMOUS, 0, "autonomy_start", now_us);
    enable_status_display(controller);
}

static command_intents_t parse_commands(command_batch_t batch,
                                        bool boot_event)
{
    command_intents_t intents = {.boot = boot_event};
    for (uint8_t i = 0; i < batch.count; ++i) {
        const char command = (char)tolower((unsigned char)batch.bytes[i]);
        switch (command) {
        case 'x': intents.stop = true; break;
        case 'f': intents.force_auto = true; break;
        case 'w': case 's': case 'a': case 'd':
        case '1': case '2': case '3': intents.motion = command; break;
        case 'r': case 't': case 'q': case 'e': case 'g':
        case 'j': case 'k':
            intents.self_test = command;
            break;
        case 'm': intents.toggle_monitor = !intents.toggle_monitor; break;
        case '+': intents.speed_delta += 50; break;
        case '-': intents.speed_delta -= 50; break;
        case 'c': intents.clear_encoder = true; break;
        case 'h': intents.help = true; break;
        case 'v': intents.enable_display = true; break;
        default: break;
        }
    }
    if (intents.stop) {
        intents.force_auto = false;
        intents.boot = false;
        intents.motion = 0;
        intents.self_test = 0;
    } else if (intents.force_auto) {
        intents.boot = false;
        intents.motion = 0;
        intents.self_test = 0;
    } else if (intents.boot) {
        intents.motion = 0;
        intents.self_test = 0;
    }
    return intents;
}

static motor_command_t manual_for(char command, int speed)
{
    switch (command) {
    case 'w': return (motor_command_t) {-speed, 0, -speed};
    case 's': return (motor_command_t) {speed, 0, speed};
    case 'a': return (motor_command_t) {-speed, speed, 0};
    case 'd': return (motor_command_t) {speed, -speed, 0};
    case '1': return (motor_command_t) {speed, 0, 0};
    case '2': return (motor_command_t) {0, speed, 0};
    case '3': return (motor_command_t) {0, 0, speed};
    default: return motor_command_zero();
    }
}

static void start_self_test(app_controller_t *controller, char command,
                            int64_t now_us)
{
    controller->mode = APP_MODE_SELF_TEST;
    controller->self_test.phase = 0;
    controller->self_test.command = command;
    if (command == 't') {
        controller->self_test.kind = SELF_TEST_DIRECT_TURN;
        controller->self_test.deadline_us = now_us + 100000;
    } else if (command == 'q' || command == 'e') {
        controller->self_test.kind = command == 'q' ?
            SELF_TEST_STRAFE_LEFT : SELF_TEST_STRAFE_RIGHT;
        controller->self_test.deadline_us = now_us +
            STRAFE_CALIBRATION_DURATION_US;
    } else if (command == 'g') {
        controller->self_test.kind = SELF_TEST_FORWARD_CALIBRATION;
        controller->self_test.deadline_us = now_us + 1000000;
    } else if (command == 'j' || command == 'k') {
        controller->self_test.kind = command == 'j' ?
            SELF_TEST_MOTOR_B_NEGATIVE : SELF_TEST_MOTOR_B_POSITIVE;
        controller->self_test.deadline_us = now_us + 1000000;
    } else {
        controller->self_test.kind = SELF_TEST_MOTOR_SEQUENCE;
        controller->self_test.deadline_us = now_us + 500000;
    }
    publish_event(controller, DIAGNOSTIC_EVENT_SELF_TEST, command, 0,
                  "self_test_start", now_us);
}

static motor_command_t self_test_step(app_controller_t *controller,
                                      int64_t now_us)
{
    if (controller->self_test.kind == SELF_TEST_STRAFE_LEFT ||
        controller->self_test.kind == SELF_TEST_STRAFE_RIGHT ||
        controller->self_test.kind == SELF_TEST_FORWARD_CALIBRATION ||
        controller->self_test.kind == SELF_TEST_MOTOR_B_NEGATIVE ||
        controller->self_test.kind == SELF_TEST_MOTOR_B_POSITIVE) {
        if (now_us >= controller->self_test.deadline_us) {
            const char command = controller->self_test.command;
            controller->self_test = (self_test_t) {0};
            controller->mode = APP_MODE_IDLE;
            publish_event(controller, DIAGNOSTIC_EVENT_SELF_TEST,
                          command, 0, "calibration_test_complete", now_us);
            return motor_command_zero();
        }
        body_motion_command_t motion = body_motion_zero();
        if (controller->self_test.kind == SELF_TEST_MOTOR_B_NEGATIVE ||
            controller->self_test.kind == SELF_TEST_MOTOR_B_POSITIVE) {
            const int direction = controller->self_test.kind ==
                SELF_TEST_MOTOR_B_NEGATIVE ? -1 : 1;
            const int magnitude = direction > 0 ?
                controller->config->kinematics.motor_b_positive_minimum :
                controller->config->obstacle.lateral_speed;
            return (motor_command_t) {
                .b = (int16_t)(direction * magnitude),
            };
        }
        if (controller->self_test.kind == SELF_TEST_FORWARD_CALIBRATION) {
            motion.forward =
                (int16_t)controller->config->obstacle.forward_speed;
        } else {
            const int direction = controller->self_test.kind ==
                                  SELF_TEST_STRAFE_LEFT ? 1 : -1;
            motion.left = (int16_t)(direction *
                controller->config->obstacle.lateral_speed);
        }
        return kiwi_inverse_kinematics(
            motion, &controller->config->kinematics);
    }
    if (controller->self_test.kind == SELF_TEST_DIRECT_TURN) {
        if (now_us >= controller->self_test.deadline_us) {
            controller->self_test = (self_test_t) {0};
            controller->mode = APP_MODE_IDLE;
            publish_event(controller, DIAGNOSTIC_EVENT_SELF_TEST,
                          't', 0, "self_test_complete", now_us);
            return motor_command_zero();
        }
        return kiwi_inverse_kinematics(
            (body_motion_command_t) {.clockwise = 420},
            &controller->config->kinematics);
    }
    static const motor_command_t commands[] = {
        {400, 0, 0}, {0, 0, 0}, {0, 400, 0},
        {0, 0, 0}, {0, 0, 400},
    };
    static const int durations_ms[] = {500, 300, 500, 300, 500};
    if (now_us >= controller->self_test.deadline_us) {
        controller->self_test.phase++;
        if (controller->self_test.phase >= 5) {
            controller->self_test = (self_test_t) {0};
            controller->mode = APP_MODE_IDLE;
            publish_event(controller, DIAGNOSTIC_EVENT_SELF_TEST,
                          'r', 0, "self_test_complete", now_us);
            return motor_command_zero();
        }
        controller->self_test.deadline_us = now_us +
            durations_ms[controller->self_test.phase] * 1000LL;
    }
    return commands[controller->self_test.phase];
}

static void apply_intents(app_controller_t *controller,
                          command_intents_t intents, int64_t now_us)
{
    controller->speed += intents.speed_delta;
    if (controller->speed > 800) controller->speed = 800;
    if (controller->speed < 100) controller->speed = 100;
    if (intents.toggle_monitor) {
        controller->line_monitor = !controller->line_monitor;
    }
    if (intents.clear_encoder) encoder_clear(&controller->encoder);
    if (intents.help) {
        publish_event(controller, DIAGNOSTIC_EVENT_INFO, 0,
                      controller->speed,
                      "q/e strafe; g forward; v display; x stop",
                      now_us);
    }
    if (intents.enable_display && !intents.stop) {
        enable_status_display(controller);
    }
    if (intents.stop) {
        controller->mode = controller->fault_bitmap == 0 ?
                           APP_MODE_IDLE : APP_MODE_FAULT;
        controller->manual_command = motor_command_zero();
        controller->self_test = (self_test_t) {0};
        publish_event(controller, DIAGNOSTIC_EVENT_STOP, 0, 0,
                      "stop", now_us);
        return;
    }
    if (intents.force_auto) {
        start_autonomy(controller, now_us, true);
    } else if (intents.boot) {
        start_autonomy(controller, now_us, false);
    } else if (intents.motion && controller->fault_bitmap == 0) {
        controller->mode = APP_MODE_MANUAL;
        controller->self_test = (self_test_t) {0};
        controller->manual_command = manual_for(intents.motion,
                                                controller->speed);
    } else if (intents.self_test && controller->fault_bitmap == 0) {
        start_self_test(controller, intents.self_test, now_us);
    }
}

static void publish_snapshot(app_controller_t *controller, int64_t now_us)
{
    const bool display_requested = controller->display_ready &&
        status_display_is_requested(&controller->display);
    if ((!controller->diagnostics_ready && !display_requested) ||
        now_us < controller->next_snapshot_us) {
        return;
    }
    const encoder_snapshot_t encoder = encoder_snapshot(&controller->encoder);
    diagnostic_snapshot_t snapshot = {
        .time_us = now_us,
        .mode = controller->mode,
        .obstacle_state = controller->obstacle.state,
        .obstacle_clear_count = controller->obstacle.clear_count,
        .line = controller->latest_line,
        .camera = controller->latest_camera,
        .line_pattern = controller->line_follow.pattern,
        .line_state = controller->line_follow.state,
        .line_error = controller->line_follow.error,
        .line_control_error = controller->line_follow.control_error,
        .line_base_speed = controller->line_follow.base_speed,
        .ultrasonic = ultrasonic_snapshot(&controller->ultrasonic),
        .motor = motor_driver_command(&controller->motor),
        .encoder_a = encoder.a,
        .encoder_b = encoder.b,
        .encoder_c = encoder.c,
        .button_raw = controller->button.raw_pressed,
        .button_stable = controller->button.stable_pressed,
        .button_armed = controller->button.armed,
        .control_overruns = controller->control_overruns,
        .diagnostic_drops = diagnostics_drop_count(&controller->diagnostics),
    };
    if (controller->diagnostics_ready) {
        diagnostics_publish_snapshot(&controller->diagnostics, &snapshot);
    }
    if (display_requested) {
        const status_display_snapshot_t display_snapshot = {
            .mode = controller->mode,
            .obstacle_state = controller->obstacle.state,
            .line_pattern = line_sensor_pattern(controller->latest_line),
            .camera_streaming = controller->latest_camera.streaming,
            .camera_frame_valid = controller->latest_camera.frame_valid,
            .camera_fresh = controller->latest_camera.fresh,
            .camera_center_permille =
                controller->latest_camera.center_permille,
            .ultrasonic_mm = snapshot.ultrasonic.filtered_mm,
            .motor = snapshot.motor,
            .finished = controller->obstacle.state ==
                        OBSTACLE_STATE_FINISHED,
            .failsafe = controller->obstacle.state ==
                        OBSTACLE_STATE_FAILSAFE,
        };
        status_display_publish(&controller->display, &display_snapshot);
    }
    controller->next_snapshot_us = now_us +
        (controller->mode == APP_MODE_AUTONOMOUS && controller->line_monitor ?
         controller->config->line_monitor_period_ms :
         controller->config->telemetry_period_ms) * 1000LL;
}

static void controller_step(app_controller_t *controller, int64_t now_us)
{
    ultrasonic_step(&controller->ultrasonic, now_us);
    controller->latest_camera = camera_line_sensor_snapshot(
        &controller->camera_line, now_us);
    controller->latest_line = controller->latest_camera.virtual_sensors;
    ultrasonic_event_t ultrasonic_event;
    const bool has_ultrasonic_event = ultrasonic_take_event(
        &controller->ultrasonic, &ultrasonic_event);
    const bool boot_event = controller->button_ready &&
        start_button_poll(&controller->button, now_us);
    const command_batch_t batch = controller->diagnostics_ready ?
        diagnostics_poll_commands(&controller->diagnostics) :
        (command_batch_t) {0};
    apply_intents(controller,
                  parse_commands(batch, boot_event),
                  now_us);

    if (controller->mode == APP_MODE_AUTONOMOUS &&
        !controller->latest_camera.fresh &&
        !controller->faults[FAULT_SOURCE_CAMERA].active) {
        set_fault(controller, FAULT_SOURCE_CAMERA, ESP_ERR_TIMEOUT,
                  FAULT_UNRECOVERABLE_THIS_BOOT);
    }

    motor_command_t final_command = motor_command_zero();
    if (controller->mode == APP_MODE_MANUAL) {
        final_command = controller->manual_command;
    } else if (controller->mode == APP_MODE_SELF_TEST) {
        final_command = self_test_step(controller, now_us);
    } else if (controller->mode == APP_MODE_AUTONOMOUS) {
        const obstacle_decision_t decision = obstacle_supervisor_step(
            &controller->obstacle,
            has_ultrasonic_event ? &ultrasonic_event : NULL,
            controller->latest_line, now_us);
        if (decision.transition != OBSTACLE_TRANSITION_NONE) {
            publish_event(controller, DIAGNOSTIC_EVENT_OBSTACLE,
                          decision.transition, decision.reason,
                          obstacle_state_name(controller->obstacle.state),
                          now_us);
        }
        if (decision.line_action == LINE_ACTION_SUSPEND) {
            line_follow_suspend(&controller->line_follow);
        } else if (decision.line_action == LINE_ACTION_RESUME) {
            line_follow_resume(&controller->line_follow);
        }
        if (decision.policy == MOTION_POLICY_LINE_FOLLOW) {
            final_command = line_follow_step(
                &controller->line_follow,
                controller->latest_line,
                controller->speed, now_us);
        } else if (decision.policy == MOTION_POLICY_OVERRIDE) {
            final_command = kiwi_inverse_kinematics(
                decision.override_motion,
                &controller->config->kinematics);
        }
    }

    const motor_result_t result = motor_driver_apply(&controller->motor,
                                                      final_command);
    if (result != MOTOR_RESULT_OK && result != MOTOR_RESULT_FAULT_LATCHED) {
        set_fault(controller, FAULT_SOURCE_MOTOR, result,
                  result == MOTOR_RESULT_RECOVERABLE_FAULT ?
                  FAULT_RECOVERABLE_EXPLICIT_REINIT :
                  FAULT_UNRECOVERABLE_THIS_BOOT);
    }
    publish_snapshot(controller, now_us);
}

static void control_task(void *arg)
{
    app_controller_t *controller = arg;
    const TickType_t period = pdMS_TO_TICKS(controller->config->control_period_ms);
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const int64_t start_us = esp_timer_get_time();
        controller_step(controller, start_us);
        const uint32_t execution_us =
            (uint32_t)(esp_timer_get_time() - start_us);
        if (execution_us > controller->max_execution_us) {
            controller->max_execution_us = execution_us;
        }
        const TickType_t now_tick = xTaskGetTickCount();
        const TickType_t next_tick = last_wake + period;
        if ((int32_t)(now_tick - next_tick) >= 0) {
            controller->control_overruns++;
            controller->missed_periods +=
                period > 0 ? (now_tick - next_tick) / period + 1 : 1;
            last_wake = now_tick;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

bool app_controller_init(app_controller_t *controller,
                         const app_config_t *config)
{
    if (controller == NULL) return false;
    memset(controller, 0, sizeof(*controller));
    controller->mode = APP_MODE_IDLE;

    const motor_hal_config_t motor_hal = motor_config();
    motor_result_t motor_result = motor_driver_preinit_safe(
        &controller->motor, &motor_hal);
    if (motor_result != MOTOR_RESULT_OK) {
        set_fault(controller, FAULT_SOURCE_MOTOR, motor_result,
                  FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    motor_result = motor_driver_init(&controller->motor);

    if (diagnostics_init(&controller->diagnostics,
                         BOARD_PIN_UART0_TX,
                         BOARD_PIN_UART0_RX) == ESP_OK) {
        controller->diagnostics_ready = true;
    } else {
        controller->degraded_bitmap |= DEGRADED_DIAGNOSTICS;
    }
    if (status_display_init(&controller->display) == ESP_OK) {
        controller->display_ready = true;
        /* Let standalone startup show camera discovery before it can fail. */
        enable_status_display(controller);
    } else {
        controller->degraded_bitmap |= DEGRADED_DISPLAY;
    }
    if (motor_result != MOTOR_RESULT_OK) {
        set_fault(controller, FAULT_SOURCE_MOTOR, motor_result,
                  motor_result == MOTOR_RESULT_RECOVERABLE_FAULT ?
                  FAULT_RECOVERABLE_EXPLICIT_REINIT :
                  FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    if (!app_config_validate(config)) {
        set_fault(controller, FAULT_SOURCE_CONTROL_CONFIG,
                  ESP_ERR_INVALID_ARG, FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    controller->config = config;
    controller->speed = config->default_speed;
    controller->line_monitor = true;
    esp_err_t result = platform_install_gpio_isr_service();
    if (result != ESP_OK) {
        set_fault(controller, FAULT_SOURCE_PLATFORM_ISR, result,
                  FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    result = camera_line_sensor_init(&controller->camera_line,
                                     &config->camera_line);
    if (result != ESP_OK) {
        set_fault(controller, FAULT_SOURCE_CAMERA, result,
                  FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    const encoder_config_t encoder_config = {
        .a_pin = {BOARD_PIN_ENCODER_A_A, BOARD_PIN_ENCODER_B_A,
                  BOARD_PIN_ENCODER_C_A},
        .b_pin = {BOARD_PIN_ENCODER_A_B, BOARD_PIN_ENCODER_B_B,
                  BOARD_PIN_ENCODER_C_B},
    };
    if (encoder_init(&controller->encoder, &encoder_config) != ESP_OK) {
        controller->degraded_bitmap |= DEGRADED_ENCODER;
    }
    const ultrasonic_driver_config_t ultrasonic_config = {
        .trigger_pin = BOARD_PIN_US_TRIG,
        .echo_pin = BOARD_PIN_US_ECHO,
        .timing = config->ultrasonic,
    };
    result = ultrasonic_init(&controller->ultrasonic, &ultrasonic_config,
                             esp_timer_get_time());
    if (result != ESP_OK) {
        set_fault(controller, FAULT_SOURCE_ULTRASONIC, result,
                  FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    result = start_button_init(&controller->button,
                               BOARD_PIN_START_BUTTON,
                               &config->button, esp_timer_get_time());
    if (result != ESP_OK) {
        set_fault(controller, FAULT_SOURCE_START_BUTTON, result,
                  FAULT_UNRECOVERABLE_THIS_BOOT);
        return false;
    }
    controller->button_ready = true;
    line_follow_init(&controller->line_follow, &config->line);
    obstacle_supervisor_init(&controller->obstacle, &config->obstacle);
    controller->latest_camera = camera_line_sensor_snapshot(
        &controller->camera_line, esp_timer_get_time());
    controller->latest_line = controller->latest_camera.virtual_sensors;

    if (controller->diagnostics_ready) {
        publish_event(controller, DIAGNOSTIC_EVENT_INFO,
                      esp_reset_reason(), 0,
                      "camera-line firmware ready; motors stopped",
                      esp_timer_get_time());
    }
    controller->initialized = true;
    return controller->fault_bitmap == 0;
}

bool app_controller_start(app_controller_t *controller)
{
    if (controller == NULL || !controller->initialized ||
        controller->task != NULL) return false;
    controller->task = xTaskCreateStatic(
        control_task, "smart_car_control", APP_CONTROL_TASK_STACK,
        controller, 5, controller->task_stack, &controller->task_buffer);
    return controller->task != NULL;
}
