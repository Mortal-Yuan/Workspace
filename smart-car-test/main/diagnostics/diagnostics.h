#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_events.h"
#include "diagnostic_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define DIAGNOSTICS_CRITICAL_DEPTH 16
#define DIAGNOSTICS_MESSAGE_MAX 384
#define DIAGNOSTICS_TASK_STACK 4096

typedef struct {
    char fault_current[DIAGNOSTICS_MESSAGE_MAX];
    size_t fault_current_len;
    size_t fault_current_offset;
    bool fault_current_valid;
    int64_t fault_current_started_us;
    int64_t fault_current_progress_us;
    char fault_next[DIAGNOSTICS_MESSAGE_MAX];
    size_t fault_next_len;
    bool fault_next_valid;
    char normal[DIAGNOSTICS_MESSAGE_MAX];
    size_t normal_len;
    size_t normal_offset;
    bool normal_valid;
    bool normal_transition;
    int64_t normal_started_us;
    int64_t normal_progress_us;
    uint32_t zero_write_count;
    uint32_t dropped_transition;
    uint32_t dropped_telemetry;
} diagnostic_transport_t;

typedef struct {
    QueueHandle_t fault_queue;
    QueueHandle_t critical_queue;
    QueueHandle_t telemetry_queue;
    StaticQueue_t fault_queue_static;
    StaticQueue_t critical_queue_static;
    StaticQueue_t telemetry_queue_static;
    uint8_t fault_storage[sizeof(fault_record_t)];
    uint8_t critical_storage[DIAGNOSTICS_CRITICAL_DEPTH *
                             sizeof(diagnostic_event_t)];
    uint8_t telemetry_storage[sizeof(diagnostic_snapshot_t)];
    diagnostic_transport_t uart;
    diagnostic_transport_t usb;
    StaticTask_t task_buffer;
    StackType_t task_stack[DIAGNOSTICS_TASK_STACK];
    TaskHandle_t task;
    portMUX_TYPE counter_lock;
    uint32_t dropped_critical;
    uint32_t dropped_transport;
    bool uart_ready;
    bool usb_ready;
    bool task_ready;
} diagnostics_t;

esp_err_t diagnostics_init(diagnostics_t *diagnostics, int uart_tx_pin,
                           int uart_rx_pin);
command_batch_t diagnostics_poll_commands(diagnostics_t *diagnostics);
void diagnostics_publish_fault(diagnostics_t *diagnostics,
                               fault_record_t fault);
void diagnostics_publish_critical(diagnostics_t *diagnostics,
                                  const diagnostic_event_t *event);
void diagnostics_publish_snapshot(diagnostics_t *diagnostics,
                                  const diagnostic_snapshot_t *snapshot);
uint32_t diagnostics_drop_count(diagnostics_t *diagnostics);
