#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_events.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "motion_types.h"

enum {
    /* LQ_TFT18SPIV33 / ILI9163B landscape drawing area. */
    STATUS_DISPLAY_WIDTH = 162,
    STATUS_DISPLAY_HEIGHT = 132,
    STATUS_DISPLAY_LINES_PER_TRANSFER = 12,
    STATUS_DISPLAY_TASK_STACK = 4096,
};

typedef struct {
    app_mode_t mode;
    uint8_t obstacle_state;
    uint8_t line_pattern;
    bool camera_streaming;
    bool camera_frame_valid;
    bool camera_fresh;
    int16_t camera_center_permille;
    int32_t ultrasonic_mm;
    motor_command_t motor;
    bool finished;
    bool failsafe;
} status_display_snapshot_t;

typedef struct {
    QueueHandle_t queue;
    StaticQueue_t queue_static;
    uint8_t queue_storage[sizeof(status_display_snapshot_t)];
    StaticTask_t task_buffer;
    StackType_t task_stack[STATUS_DISPLAY_TASK_STACK];
    TaskHandle_t task;
    uint16_t pixel_buffer[STATUS_DISPLAY_WIDTH *
                          STATUS_DISPLAY_LINES_PER_TRANSFER];
    volatile bool requested;
    volatile bool active;
    volatile esp_err_t last_error;
} status_display_t;

esp_err_t status_display_init(status_display_t *display);
bool status_display_enable(status_display_t *display);
void status_display_publish(status_display_t *display,
                            const status_display_snapshot_t *snapshot);
bool status_display_is_requested(const status_display_t *display);
bool status_display_is_active(const status_display_t *display);
esp_err_t status_display_last_error(const status_display_t *display);
