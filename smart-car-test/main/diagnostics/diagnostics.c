#include "diagnostics.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_timer.h"

enum {
    DIAGNOSTICS_NORMAL_BAUD = 115200,
    DIAGNOSTICS_PREVIEW_BAUD = 460800,
    DIAGNOSTICS_PREVIEW_TIMEOUT_US = 3000000,
    DIAGNOSTICS_UART_TX_BUFFER = 8192,
};

static const char PREVIEW_READY[] =
    "USB_PREVIEW_READY baud=460800 version=1\n";
static const char PREVIEW_STOPPED[] =
    "USB_PREVIEW_STOPPED baud=115200\n";

static const char *mode_name(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_IDLE: return "IDLE";
    case APP_MODE_AUTONOMOUS: return "AUTO";
    case APP_MODE_MANUAL: return "MANUAL";
    case APP_MODE_SELF_TEST: return "SELF_TEST";
    case APP_MODE_BALL_APPROACH: return "BALL";
    case APP_MODE_BALL_MISSION: return "MISSION";
    case APP_MODE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

static int transport_write(const char *data, size_t length)
{
    const size_t budget = length > 256 ? 256 : length;
    return uart_write_bytes(UART_NUM_0, data, budget);
}

static void apply_preview_mode(diagnostics_t *diagnostics, int64_t now_us)
{
    bool active = false;
    bool request_pending = false;
    bool requested_state = false;
    portENTER_CRITICAL(&diagnostics->preview_lock);
    active = diagnostics->preview_active;
    if (active && now_us - diagnostics->preview_keepalive_us >
                      DIAGNOSTICS_PREVIEW_TIMEOUT_US) {
        diagnostics->preview_request_pending = true;
        diagnostics->preview_requested_state = false;
    }
    request_pending = diagnostics->preview_request_pending;
    requested_state = diagnostics->preview_requested_state;
    portEXIT_CRITICAL(&diagnostics->preview_lock);
    if (!request_pending) return;

    if (requested_state == active) {
        portENTER_CRITICAL(&diagnostics->preview_lock);
        diagnostics->preview_request_pending = false;
        portEXIT_CRITICAL(&diagnostics->preview_lock);
        return;
    }

    /* Do not carry a partial low-rate telemetry line across a baud change. */
    diagnostics->uart.normal_valid = false;
    (void)uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(500));
    if (requested_state) {
        (void)uart_write_bytes(UART_NUM_0, PREVIEW_READY,
                               sizeof(PREVIEW_READY) - 1U);
        (void)uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(500));
        if (uart_set_baudrate(UART_NUM_0, DIAGNOSTICS_PREVIEW_BAUD) == ESP_OK) {
            portENTER_CRITICAL(&diagnostics->preview_lock);
            diagnostics->preview_active = true;
            diagnostics->preview_request_pending = false;
            diagnostics->preview_keepalive_us = now_us;
            portEXIT_CRITICAL(&diagnostics->preview_lock);
        }
    } else {
        /* Stop new binary writes before draining and changing back. */
        portENTER_CRITICAL(&diagnostics->preview_lock);
        diagnostics->preview_active = false;
        diagnostics->preview_request_pending = false;
        portEXIT_CRITICAL(&diagnostics->preview_lock);
        (void)uart_write_bytes(UART_NUM_0, PREVIEW_STOPPED,
                               sizeof(PREVIEW_STOPPED) - 1U);
        (void)uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(500));
        (void)uart_set_baudrate(UART_NUM_0, DIAGNOSTICS_NORMAL_BAUD);
    }
}

static void promote_fault(diagnostic_transport_t *transport)
{
    if (!transport->fault_current_valid && transport->fault_next_valid) {
        memcpy(transport->fault_current, transport->fault_next,
               transport->fault_next_len);
        transport->fault_current_len = transport->fault_next_len;
        transport->fault_current_offset = 0;
        transport->fault_current_valid = true;
        transport->fault_next_valid = false;
        transport->fault_current_started_us = esp_timer_get_time();
        transport->fault_current_progress_us =
            transport->fault_current_started_us;
    }
}

static void enqueue_fault(diagnostic_transport_t *transport,
                          const char *message, size_t length)
{
    if (!transport->fault_current_valid) {
        memcpy(transport->fault_current, message, length);
        transport->fault_current_len = length;
        transport->fault_current_offset = 0;
        transport->fault_current_valid = true;
        transport->fault_current_started_us = esp_timer_get_time();
        transport->fault_current_progress_us =
            transport->fault_current_started_us;
    } else if (transport->fault_current_offset == 0) {
        memcpy(transport->fault_current, message, length);
        transport->fault_current_len = length;
    } else {
        memcpy(transport->fault_next, message, length);
        transport->fault_next_len = length;
        transport->fault_next_valid = true;
    }
}

static void record_transport_drop(diagnostics_t *diagnostics)
{
    portENTER_CRITICAL(&diagnostics->counter_lock);
    diagnostics->dropped_transport++;
    portEXIT_CRITICAL(&diagnostics->counter_lock);
}

static void enqueue_normal(diagnostics_t *diagnostics,
                           diagnostic_transport_t *transport,
                           const char *message, size_t length,
                           bool transition)
{
    if (transport->normal_valid) {
        if (transition && !transport->normal_transition &&
            transport->normal_offset == 0) {
            memcpy(transport->normal, message, length);
            transport->normal_len = length;
            transport->normal_transition = true;
            transport->normal_started_us = esp_timer_get_time();
            transport->normal_progress_us =
                transport->normal_started_us;
            record_transport_drop(diagnostics);
            return;
        }
        if (transition) {
            transport->dropped_transition++;
        } else {
            transport->dropped_telemetry++;
        }
        record_transport_drop(diagnostics);
        return;
    }
    memcpy(transport->normal, message, length);
    transport->normal_len = length;
    transport->normal_offset = 0;
    transport->normal_valid = true;
    transport->normal_transition = transition;
    transport->normal_started_us = esp_timer_get_time();
    transport->normal_progress_us = transport->normal_started_us;
}

static void pump_transport(diagnostic_transport_t *transport, int64_t now_us)
{
    promote_fault(transport);
    char *data = NULL;
    size_t *offset = NULL;
    size_t length = 0;
    bool fault = false;
    if (transport->fault_current_valid) {
        data = transport->fault_current;
        offset = &transport->fault_current_offset;
        length = transport->fault_current_len;
        fault = true;
        if (now_us - transport->fault_current_started_us > 5000000LL ||
            now_us - transport->fault_current_progress_us > 2000000LL) {
            transport->fault_current_valid = false;
            promote_fault(transport);
            return;
        }
    } else if (transport->normal_valid) {
        data = transport->normal;
        offset = &transport->normal_offset;
        length = transport->normal_len;
        if (now_us - transport->normal_started_us > 1000000LL ||
            now_us - transport->normal_progress_us > 500000LL) {
            transport->normal_valid = false;
            return;
        }
    } else {
        return;
    }
    const int written = transport_write(data + *offset, length - *offset);
    if (written > 0) {
        *offset += (size_t)written;
        if (fault) {
            transport->fault_current_progress_us = now_us;
        } else {
            transport->normal_progress_us = now_us;
        }
        if (*offset >= length) {
            if (fault) {
                transport->fault_current_valid = false;
                promote_fault(transport);
            } else {
                transport->normal_valid = false;
            }
        }
    } else {
        transport->zero_write_count++;
    }
}

static size_t bounded_length(int result, size_t capacity)
{
    if (result <= 0) return 0;
    return (size_t)result < capacity ? (size_t)result : capacity - 1;
}

static void diagnostics_task(void *arg)
{
    diagnostics_t *diagnostics = arg;
    char scratch[DIAGNOSTICS_MESSAGE_MAX];
    while (true) {
        apply_preview_mode(diagnostics, esp_timer_get_time());
        fault_record_t fault;
        diagnostic_event_t event;
        diagnostic_snapshot_t snapshot;
        size_t length = 0;
        bool is_fault = false;
        bool transition = false;
        if (xQueueReceive(diagnostics->fault_queue, &fault, 0) == pdTRUE) {
            length = bounded_length(snprintf(
                scratch, sizeof(scratch),
                "FAULT source=%d code=%" PRId32 " recoverability=%d active=%d\n",
                fault.source, fault.code, fault.recoverability, fault.active),
                sizeof(scratch));
            is_fault = true;
        } else if (xQueueReceive(diagnostics->critical_queue, &event, 0) ==
                   pdTRUE) {
            length = bounded_length(snprintf(
                scratch, sizeof(scratch),
                "EVENT t=%" PRId64 "ms kind=%d code=%" PRId32
                " value=%" PRId32 " %s\n",
                event.time_us / 1000, event.kind, event.code,
                event.value, event.text), sizeof(scratch));
            transition = true;
        } else if (xQueueReceive(diagnostics->telemetry_queue, &snapshot, 0) ==
                   pdTRUE) {
            length = bounded_length(snprintf(
                scratch, sizeof(scratch),
                "STATUS t=%" PRId64 "ms mode=%s obstacle=%u progress=%u "
                "startup=%u mission=%u ballctl=%u/%u/%d/%u/%u/%u "
                "CAM=%d%d%d%d pattern=%x fresh=%d seq=%" PRIu32
                " pos=%d far=%d head=%d steer=%d width=%u "
                "comp=%u/%u/%u black=%u thr=%u contrast=%u "
                "cam_drop=%" PRIu32 " cam_err=%" PRIu32
                " BALL=%u/%d/%d/%u bfar=%d bxy=%d/%u bwh=%u/%u ba=%u "
                "bshape=%u/%u bconf=%u brgb=%u/%u/%u bpix=%u "
                "bseed=%u/%u bbox=%u/%u/%u/%u bc=%u "
                "bprobe=%d/%d/%u/%u/%u/%u "
                "GREEN=%u/%d/%d/%u gfar=%d gxy=%d/%u ga=%u "
                "gconf=%u grgb=%u/%u/%u gpix=%u/%u "
                "gbbox=%u/%u/%u/%u "
                "LEFT_TARGET=%u/%d/%d/%u lfar=%d lxy=%d/%u "
                "lwh=%u/%u la=%u lconf=%u lrgb=%u/%u/%u "
                "lpix=%u/%u lbbox=%u/%u/%u/%u "
                "RIGHT_TARGET=%u/%d/%d/%u rfar=%d rxy=%d/%u "
                "rwh=%u/%u ra=%u rconf=%u rrgb=%u/%u/%u "
                "rpix=%u/%u rbbox=%u/%u/%u/%u "
                " line=%d err=%d/%d base=%d "
                "us=%" PRId32 "/%" PRId32
                " q=%d echo=%d wait=%d timeout=%" PRIu32
                " anomaly=%" PRIu32 " cmd=%d,%d,%d "
                "button=%d/%d/%d encoder=%" PRId64 ",%" PRId64
                ",%" PRId64 " overrun=%" PRIu32 " drops=%" PRIu32 "\n",
                snapshot.time_us / 1000, mode_name(snapshot.mode),
                snapshot.obstacle_state, snapshot.obstacle_clear_count,
                snapshot.startup_maneuver_phase,
                snapshot.ball_mission_state,
                snapshot.ball_approach_state,
                snapshot.ball_target_color,
                snapshot.ball_approach_error,
                snapshot.ball_capture_frames,
                snapshot.ball_goal_frames,
                snapshot.ball_approach_reason,
                snapshot.line.left, snapshot.line.left_center,
                snapshot.line.right_center, snapshot.line.right,
                (unsigned)snapshot.line_pattern, snapshot.camera.fresh,
                snapshot.camera.decoded_frames,
                snapshot.camera.center_permille,
                snapshot.camera.far_center_permille,
                snapshot.camera.heading_permille,
                snapshot.camera.steering_permille,
                (unsigned)snapshot.camera.width_permille,
                (unsigned)snapshot.camera.connected_component_count,
                (unsigned)snapshot.camera.component_height_permille,
                (unsigned)snapshot.camera.component_area_permille,
                (unsigned)snapshot.camera.black_permille,
                (unsigned)snapshot.camera.threshold,
                (unsigned)snapshot.camera.contrast,
                snapshot.camera.dropped_frames,
                snapshot.camera.decode_errors,
                (unsigned)snapshot.camera.ball.color,
                snapshot.camera.ball.candidate,
                snapshot.camera.ball.detected,
                (unsigned)snapshot.camera.ball.stable_frames,
                snapshot.camera.ball.far_candidate,
                snapshot.camera.ball.center_x_permille,
                (unsigned)snapshot.camera.ball.center_y_permille,
                (unsigned)snapshot.camera.ball.width_permille,
                (unsigned)snapshot.camera.ball.height_permille,
                (unsigned)snapshot.camera.ball.area_permille,
                (unsigned)snapshot.camera.ball.fill_permille,
                (unsigned)snapshot.camera.ball.roundness_permille,
                (unsigned)snapshot.camera.ball.confidence_permille,
                (unsigned)snapshot.camera.ball.mean_red,
                (unsigned)snapshot.camera.ball.mean_green,
                (unsigned)snapshot.camera.ball.mean_blue,
                (unsigned)snapshot.camera.ball.matched_pixels,
                (unsigned)snapshot.camera.ball.strong_pixels,
                (unsigned)snapshot.camera.ball.highlight_pixels,
                (unsigned)snapshot.camera.ball.box_left_permille,
                (unsigned)snapshot.camera.ball.box_top_permille,
                (unsigned)snapshot.camera.ball.box_right_permille,
                (unsigned)snapshot.camera.ball.box_bottom_permille,
                (unsigned)snapshot.camera.ball.component_count,
                snapshot.camera.ball.probe_red_score,
                snapshot.camera.ball.probe_x_permille,
                (unsigned)snapshot.camera.ball.probe_y_permille,
                (unsigned)snapshot.camera.ball.probe_red,
                (unsigned)snapshot.camera.ball.probe_green,
                (unsigned)snapshot.camera.ball.probe_blue,
                (unsigned)snapshot.camera.green_ball.color,
                snapshot.camera.green_ball.candidate,
                snapshot.camera.green_ball.detected,
                (unsigned)snapshot.camera.green_ball.stable_frames,
                snapshot.camera.green_ball.far_candidate,
                snapshot.camera.green_ball.center_x_permille,
                (unsigned)snapshot.camera.green_ball.center_y_permille,
                (unsigned)snapshot.camera.green_ball.area_permille,
                (unsigned)snapshot.camera.green_ball.confidence_permille,
                (unsigned)snapshot.camera.green_ball.mean_red,
                (unsigned)snapshot.camera.green_ball.mean_green,
                (unsigned)snapshot.camera.green_ball.mean_blue,
                (unsigned)snapshot.camera.green_ball.matched_pixels,
                (unsigned)snapshot.camera.green_ball.strong_pixels,
                (unsigned)snapshot.camera.green_ball.box_left_permille,
                (unsigned)snapshot.camera.green_ball.box_top_permille,
                (unsigned)snapshot.camera.green_ball.box_right_permille,
                (unsigned)snapshot.camera.green_ball.box_bottom_permille,
                (unsigned)snapshot.camera.left_target.color,
                snapshot.camera.left_target.candidate,
                snapshot.camera.left_target.detected,
                (unsigned)snapshot.camera.left_target.stable_frames,
                snapshot.camera.left_target.far_candidate,
                snapshot.camera.left_target.center_x_permille,
                (unsigned)snapshot.camera.left_target.center_y_permille,
                (unsigned)snapshot.camera.left_target.width_permille,
                (unsigned)snapshot.camera.left_target.height_permille,
                (unsigned)snapshot.camera.left_target.area_permille,
                (unsigned)snapshot.camera.left_target.confidence_permille,
                (unsigned)snapshot.camera.left_target.mean_red,
                (unsigned)snapshot.camera.left_target.mean_green,
                (unsigned)snapshot.camera.left_target.mean_blue,
                (unsigned)snapshot.camera.left_target.matched_pixels,
                (unsigned)snapshot.camera.left_target.strong_pixels,
                (unsigned)snapshot.camera.left_target.box_left_permille,
                (unsigned)snapshot.camera.left_target.box_top_permille,
                (unsigned)snapshot.camera.left_target.box_right_permille,
                (unsigned)snapshot.camera.left_target.box_bottom_permille,
                (unsigned)snapshot.camera.right_target.color,
                snapshot.camera.right_target.candidate,
                snapshot.camera.right_target.detected,
                (unsigned)snapshot.camera.right_target.stable_frames,
                snapshot.camera.right_target.far_candidate,
                snapshot.camera.right_target.center_x_permille,
                (unsigned)snapshot.camera.right_target.center_y_permille,
                (unsigned)snapshot.camera.right_target.width_permille,
                (unsigned)snapshot.camera.right_target.height_permille,
                (unsigned)snapshot.camera.right_target.area_permille,
                (unsigned)snapshot.camera.right_target.confidence_permille,
                (unsigned)snapshot.camera.right_target.mean_red,
                (unsigned)snapshot.camera.right_target.mean_green,
                (unsigned)snapshot.camera.right_target.mean_blue,
                (unsigned)snapshot.camera.right_target.matched_pixels,
                (unsigned)snapshot.camera.right_target.strong_pixels,
                (unsigned)snapshot.camera.right_target.box_left_permille,
                (unsigned)snapshot.camera.right_target.box_top_permille,
                (unsigned)snapshot.camera.right_target.box_right_permille,
                (unsigned)snapshot.camera.right_target.box_bottom_permille,
                snapshot.line_state,
                snapshot.line_error, snapshot.line_control_error,
                snapshot.line_base_speed,
                snapshot.ultrasonic.raw_mm,
                snapshot.ultrasonic.filtered_mm,
                snapshot.ultrasonic.quality, snapshot.ultrasonic.echo_high,
                snapshot.ultrasonic.waiting,
                snapshot.ultrasonic.timeout_count,
                snapshot.ultrasonic.anomaly_count,
                snapshot.motor.a, snapshot.motor.b, snapshot.motor.c,
                snapshot.button_raw, snapshot.button_stable,
                snapshot.button_armed, snapshot.encoder_a,
                snapshot.encoder_b, snapshot.encoder_c,
                snapshot.control_overruns, snapshot.diagnostic_drops),
                sizeof(scratch));
        }
        if (length > 0) {
            if (is_fault) {
                if (diagnostics->uart_ready) {
                    enqueue_fault(&diagnostics->uart, scratch, length);
                }
            } else {
                if (diagnostics->uart_ready) {
                    enqueue_normal(diagnostics, &diagnostics->uart,
                                   scratch, length,
                                   transition);
                }
            }
        }
        const int64_t now_us = esp_timer_get_time();
        if (diagnostics->uart_ready) {
            pump_transport(&diagnostics->uart, now_us);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t diagnostics_init(diagnostics_t *diagnostics, int uart_tx_pin,
                           int uart_rx_pin)
{
    if (diagnostics == NULL) return ESP_ERR_INVALID_ARG;
    memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->counter_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    diagnostics->preview_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    diagnostics->fault_queue = xQueueCreateStatic(
        1, sizeof(fault_record_t), diagnostics->fault_storage,
        &diagnostics->fault_queue_static);
    diagnostics->critical_queue = xQueueCreateStatic(
        DIAGNOSTICS_CRITICAL_DEPTH, sizeof(diagnostic_event_t),
        diagnostics->critical_storage, &diagnostics->critical_queue_static);
    diagnostics->telemetry_queue = xQueueCreateStatic(
        1, sizeof(diagnostic_snapshot_t), diagnostics->telemetry_storage,
        &diagnostics->telemetry_queue_static);
    if (!diagnostics->fault_queue || !diagnostics->critical_queue ||
        !diagnostics->telemetry_queue) {
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t uart_config = {
        .baud_rate = DIAGNOSTICS_NORMAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(UART_NUM_0, &uart_config) == ESP_OK &&
        uart_set_pin(UART_NUM_0, uart_tx_pin, uart_rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK &&
        uart_driver_install(UART_NUM_0, 512, DIAGNOSTICS_UART_TX_BUFFER,
                            0, NULL, 0) == ESP_OK) {
        diagnostics->uart_ready = true;
    }
    diagnostics->task = xTaskCreateStatic(
        diagnostics_task, "diagnostics", DIAGNOSTICS_TASK_STACK,
        diagnostics, 2, diagnostics->task_stack, &diagnostics->task_buffer);
    diagnostics->task_ready = diagnostics->task != NULL;
    return diagnostics->task_ready ? ESP_OK : ESP_ERR_NO_MEM;
}

command_batch_t diagnostics_poll_commands(diagnostics_t *diagnostics)
{
    command_batch_t batch = {0};
    if (diagnostics == NULL) return batch;
    if (diagnostics->uart_ready) {
        const int count = uart_read_bytes(
            UART_NUM_0, batch.bytes + batch.count,
            APP_COMMAND_BATCH_MAX - batch.count, 0);
        if (count > 0) batch.count += (uint8_t)count;
    }
    return batch;
}

void diagnostics_publish_fault(diagnostics_t *diagnostics,
                               fault_record_t fault)
{
    if (diagnostics && diagnostics->fault_queue) {
        xQueueOverwrite(diagnostics->fault_queue, &fault);
    }
}

void diagnostics_publish_critical(diagnostics_t *diagnostics,
                                  const diagnostic_event_t *event)
{
    if (!diagnostics || !event || !diagnostics->critical_queue) return;
    if (xQueueSend(diagnostics->critical_queue, event, 0) != pdTRUE) {
        portENTER_CRITICAL(&diagnostics->counter_lock);
        diagnostics->dropped_critical++;
        portEXIT_CRITICAL(&diagnostics->counter_lock);
    }
}

void diagnostics_publish_snapshot(diagnostics_t *diagnostics,
                                  const diagnostic_snapshot_t *snapshot)
{
    if (diagnostics && snapshot && diagnostics->telemetry_queue) {
        xQueueOverwrite(diagnostics->telemetry_queue, snapshot);
    }
}

void diagnostics_request_usb_preview(diagnostics_t *diagnostics,
                                     bool enabled)
{
    if (diagnostics == NULL || !diagnostics->uart_ready) return;
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&diagnostics->preview_lock);
    if (enabled) diagnostics->preview_keepalive_us = now_us;
    if (diagnostics->preview_active != enabled) {
        diagnostics->preview_requested_state = enabled;
        diagnostics->preview_request_pending = true;
    }
    portEXIT_CRITICAL(&diagnostics->preview_lock);
}

void diagnostics_write_camera_preview(
    void *context, const camera_preview_packet_t *packet)
{
    diagnostics_t *diagnostics = context;
    if (diagnostics == NULL || packet == NULL || !diagnostics->uart_ready) {
        return;
    }
    bool active = false;
    portENTER_CRITICAL(&diagnostics->preview_lock);
    active = diagnostics->preview_active;
    portEXIT_CRITICAL(&diagnostics->preview_lock);
    if (active) {
        /* One driver call preserves packet ordering relative to text writers. */
        (void)uart_write_bytes(UART_NUM_0, packet, sizeof(*packet));
    }
}

uint32_t diagnostics_drop_count(diagnostics_t *diagnostics)
{
    if (!diagnostics) return 0;
    portENTER_CRITICAL(&diagnostics->counter_lock);
    const uint32_t result = diagnostics->dropped_critical +
                            diagnostics->dropped_transport;
    portEXIT_CRITICAL(&diagnostics->counter_lock);
    return result;
}
