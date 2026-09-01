#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "camera_ball_vision.h"
#include "camera_line_vision.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sensor_types.h"
#include "usb/uvc_host.h"

enum {
    CAMERA_LINE_USB_TASK_STACK = 4096,
    CAMERA_LINE_DECODE_TASK_STACK = 8192,
    CAMERA_LINE_OUTPUT_WIDTH = 80,
    CAMERA_LINE_OUTPUT_HEIGHT = 60,
};

typedef struct {
    camera_line_config_t config;
    camera_ball_config_t ball_config;
    QueueHandle_t frame_queue;
    StaticQueue_t frame_queue_static;
    uint8_t frame_queue_storage[sizeof(uvc_host_frame_t *)];
    StaticTask_t usb_task_buffer;
    StackType_t usb_task_stack[CAMERA_LINE_USB_TASK_STACK];
    TaskHandle_t usb_task;
    StaticTask_t decode_task_buffer;
    StackType_t decode_task_stack[CAMERA_LINE_DECODE_TASK_STACK];
    TaskHandle_t decode_task;
    uvc_host_stream_hdl_t stream;
    uint8_t *rgb_buffer;
    size_t rgb_buffer_size;
    camera_line_vision_workspace_t *vision_workspace;
    camera_ball_vision_workspace_t *ball_workspace;
    unsigned stream_width;
    unsigned stream_height;
    float stream_fps;
    portMUX_TYPE lock;
    camera_line_snapshot_t snapshot;
    uint8_t normal_line_frames;
    uint8_t missing_line_frames;
    uint8_t finish_candidate_frames;
    bool finish_detection_enabled;
    bool history_valid;
    int16_t history_center_permille;
    int16_t history_steering_permille;
    uint8_t ball_stable_frames;
    ball_color_t ball_stable_color;
    int16_t ball_track_x_permille;
    uint16_t ball_track_y_permille;
    bool ascii_view_requested;
    bool initialized;
} camera_line_sensor_t;

esp_err_t camera_line_sensor_init(camera_line_sensor_t *sensor,
                                  const camera_line_config_t *config,
                                  const camera_ball_config_t *ball_config);
camera_line_snapshot_t camera_line_sensor_snapshot(
    camera_line_sensor_t *sensor, int64_t now_us);
bool camera_line_sensor_request_ascii_view(camera_line_sensor_t *sensor);
void camera_line_sensor_set_finish_detection_enabled(
    camera_line_sensor_t *sensor, bool enabled);
