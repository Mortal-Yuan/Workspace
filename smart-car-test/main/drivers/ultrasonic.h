#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "sensor_types.h"

#define ULTRASONIC_MEDIAN_WINDOW 3

typedef struct {
    int trigger_pin;
    int echo_pin;
    ultrasonic_config_t timing;
} ultrasonic_driver_config_t;

typedef struct {
    ultrasonic_driver_config_t config;
    portMUX_TYPE lock;
    uint32_t session_generation;
    uint32_t trigger_token;
    uint32_t captured_session;
    uint32_t captured_token;
    bool active;
    bool rise_seen;
    bool capture_complete;
    bool anomaly;
    int64_t rise_us;
    int64_t pulse_us;
    int64_t deadline_us;
    int64_t next_trigger_us;
    bool blocked_echo_high;
    bool event_pending;
    ultrasonic_event_t pending_event;
    uint32_t next_seq;
    int window[ULTRASONIC_MEDIAN_WINDOW];
    unsigned window_count;
    unsigned window_next;
    int outlier_candidate_mm;
    unsigned outlier_count;
    unsigned timeout_count;
    uint32_t anomaly_count;
    ultrasonic_snapshot_t snapshot;
    bool initialized;
} ultrasonic_t;

esp_err_t ultrasonic_init(ultrasonic_t *sensor,
                          const ultrasonic_driver_config_t *config,
                          int64_t now_us);
void ultrasonic_restart_session(ultrasonic_t *sensor, int64_t now_us);
void ultrasonic_step(ultrasonic_t *sensor, int64_t now_us);
bool ultrasonic_take_event(ultrasonic_t *sensor,
                           ultrasonic_event_t *event);
ultrasonic_snapshot_t ultrasonic_snapshot(const ultrasonic_t *sensor);
