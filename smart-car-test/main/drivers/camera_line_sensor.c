#include "camera_line_sensor.h"

#include <stddef.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"
#include "usb/usb_host.h"

enum {
    CAMERA_USB_VID = 0x349c,
    CAMERA_USB_PID = 0x3307,
    CAMERA_FRAME_BUFFER_BYTES = 512 * 1024,
    CAMERA_URB_BYTES = 10 * 1024,
    CAMERA_PREVIEW_PERIOD_US = 200000,
};

typedef struct {
    unsigned width;
    unsigned height;
    float fps;
    esp_jpeg_image_scale_t decode_scale;
    const char *name;
} camera_stream_profile_t;

static const camera_stream_profile_t CAMERA_PROFILES[] = {
    {640, 480, 15.0f, JPEG_IMAGE_SCALE_1_8,
     "MJPEG 640x480 @ 15 fps, 80x60 decode"},
};

_Static_assert((int)CAMERA_LINE_OUTPUT_WIDTH <=
                   (int)CAMERA_BALL_VISION_MAX_WIDTH,
               "ball workspace is smaller than the decoded camera frame");
_Static_assert((int)CAMERA_LINE_OUTPUT_HEIGHT <=
                   (int)CAMERA_BALL_VISION_MAX_HEIGHT,
               "ball workspace is smaller than the decoded camera frame");
_Static_assert((int)CAMERA_LINE_OUTPUT_HEIGHT / 2 <=
                   (int)CAMERA_LINE_VISION_MAX_HEIGHT,
               "line workspace is shorter than the lower camera half");
_Static_assert(CAMERA_LINE_OUTPUT_WIDTH *
                   (CAMERA_LINE_OUTPUT_HEIGHT / 2) <=
                   CAMERA_LINE_VISION_MAX_ROI_PIXELS,
               "line workspace is smaller than the lower camera half");

static const char *TAG = "camera_line";

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static void dump_ascii_view(const uint8_t *pixels, size_t width,
                            size_t height)
{
    static const char SHADES[] = "@%#*+=-:.";
    if (pixels == NULL || width > CAMERA_LINE_OUTPUT_WIDTH ||
        height > CAMERA_LINE_OUTPUT_HEIGHT) {
        return;
    }

    ESP_LOGI(TAG, "VIEW_BEGIN %ux%u", (unsigned)width, (unsigned)height);
    for (size_t y = 0; y < height; ++y) {
        char row[CAMERA_LINE_OUTPUT_WIDTH + 1];
        for (size_t x = 0; x < width; ++x) {
            const uint8_t *rgb = pixels + (y * width + x) * 3U;
            const unsigned gray =
                (77U * rgb[0] + 150U * rgb[1] + 29U * rgb[2]) >> 8;
            row[x] = SHADES[gray * (sizeof(SHADES) - 2U) / 255U];
        }
        row[width] = '\0';
        ESP_LOGI(TAG, "VIEW%02u:%s", (unsigned)y, row);
    }
    ESP_LOGI(TAG, "VIEW_END");
}

static const camera_stream_profile_t *active_profile(
    const camera_line_sensor_t *sensor)
{
    for (size_t index = 0;
         index < sizeof(CAMERA_PROFILES) / sizeof(CAMERA_PROFILES[0]);
         ++index) {
        if (CAMERA_PROFILES[index].width == sensor->stream_width &&
            CAMERA_PROFILES[index].height == sensor->stream_height) {
            return &CAMERA_PROFILES[index];
        }
    }
    return NULL;
}

static void update_streaming(camera_line_sensor_t *sensor, bool streaming)
{
    portENTER_CRITICAL(&sensor->lock);
    sensor->snapshot.streaming = streaming;
    portEXIT_CRITICAL(&sensor->lock);
}

static bool camera_frame_callback(const uvc_host_frame_t *frame,
                                  void *user_context)
{
    camera_line_sensor_t *sensor = user_context;
    if (sensor == NULL || frame == NULL || frame->data == NULL ||
        frame->data_len == 0) {
        return true;
    }

    portENTER_CRITICAL(&sensor->lock);
    ++sensor->snapshot.received_frames;
    portEXIT_CRITICAL(&sensor->lock);

    uvc_host_frame_t *queued_frame = (uvc_host_frame_t *)frame;
    if (xQueueSendToBack(sensor->frame_queue, &queued_frame, 0) != pdPASS) {
        portENTER_CRITICAL(&sensor->lock);
        ++sensor->snapshot.dropped_frames;
        portEXIT_CRITICAL(&sensor->lock);
        return true;
    }
    return false;
}

static void camera_stream_event_callback(
    const uvc_host_stream_event_data_t *event, void *user_context)
{
    camera_line_sensor_t *sensor = user_context;
    if (sensor == NULL || event == NULL) return;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "UVC transfer error: %s",
                 esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        update_streaming(sensor, false);
        ESP_LOGE(TAG, "camera disconnected; autonomous motion will stop");
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        portENTER_CRITICAL(&sensor->lock);
        ++sensor->snapshot.dropped_frames;
        portEXIT_CRITICAL(&sensor->lock);
        break;
    default:
        break;
    }
}

static void usb_host_event_task(void *argument)
{
    camera_line_sensor_t *sensor = argument;
    while (true) {
        uint32_t event_flags = 0;
        const esp_err_t result = usb_host_lib_handle_events(
            portMAX_DELAY, &event_flags);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "USB Host event error: %s",
                     esp_err_to_name(result));
            continue;
        }
        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0) {
            usb_host_device_free_all();
        }
        (void)sensor;
    }
}

static camera_ball_observation_t confirm_target(
    camera_line_sensor_t *sensor, camera_ball_observation_t target,
    camera_target_tracker_t *tracker)
{
    if (target.candidate) {
        const int delta_x = target.center_x_permille -
            tracker->track_x_permille;
        const int delta_y = (int)target.center_y_permille -
            tracker->track_y_permille;
        const bool same_track = tracker->stable_frames > 0U &&
            (delta_x < 0 ? -delta_x : delta_x) <=
                sensor->ball_config.tracking_tolerance_permille &&
            (delta_y < 0 ? -delta_y : delta_y) <=
                sensor->ball_config.tracking_tolerance_permille;
        if (same_track) {
            if (tracker->stable_frames < UINT8_MAX) {
                ++tracker->stable_frames;
            }
        } else {
            tracker->stable_frames = 1;
        }
        tracker->track_x_permille = target.center_x_permille;
        tracker->track_y_permille = target.center_y_permille;
    } else {
        tracker->stable_frames = 0;
    }
    target.stable_frames = tracker->stable_frames;
    const int required_frames = target.color == BALL_COLOR_GREEN ?
        sensor->ball_config.green_confirm_frames :
        target.color == BALL_COLOR_BLUE ?
            sensor->ball_config.blue_target_confirm_frames :
        target.far_candidate ? sensor->ball_config.far_confirm_frames :
                               sensor->ball_config.confirm_frames;
    target.detected = target.candidate &&
        target.stable_frames >= required_frames;
    return target;
}

static void publish_analysis(
    camera_line_sensor_t *sensor, camera_line_analysis_t analysis,
    camera_ball_observation_t red_ball,
    camera_ball_observation_t green_ball,
    camera_ball_observation_t left_target,
    camera_ball_observation_t right_target, int64_t now_us)
{
    portENTER_CRITICAL(&sensor->lock);
    const bool reliable_history_candidate =
        analysis.line_detected && !analysis.finish_detected;
    const bool raw_finish_shape = analysis.finish_detected;
    const bool raw_finish_candidate = raw_finish_shape &&
        sensor->finish_detection_enabled;
    const bool finish_confirmed = camera_line_finish_confirmed(
        raw_finish_candidate,
        sensor->config.finish_confirm_frames,
        &sensor->finish_candidate_frames);
    const bool finish_suppressed = raw_finish_shape && !finish_confirmed;
    if (finish_suppressed) {
        /* A wide dark object while the runtime gate is closed, or for only
         * one decoded frame after it opens, is not a finish line.  It remains
         * a valid connected line while confirmation is incomplete. */
        analysis.finish_detected = false;
        analysis.virtual_sensors = camera_line_virtual_sensors(
            analysis.steering_permille, false);
    }
    if (analysis.line_detected && sensor->snapshot.line_detected &&
        !analysis.finish_detected) {
        analysis.center_permille = (int16_t)(
            (2 * sensor->snapshot.center_permille +
             analysis.center_permille) / 3);
        analysis.far_center_permille = (int16_t)(
            (2 * sensor->snapshot.far_center_permille +
             analysis.far_center_permille) / 3);
        analysis.heading_permille = (int16_t)clamp_int(
            analysis.far_center_permille - analysis.center_permille,
            -2000, 2000);
        analysis.steering_permille = (int16_t)clamp_int(
            camera_line_steering_from_geometry(
                analysis.center_permille, analysis.far_center_permille,
                &sensor->config),
            -1000, 1000);
        analysis.virtual_sensors = camera_line_virtual_sensors(
            analysis.steering_permille, false);
    }
    if (analysis.line_detected) {
        sensor->missing_line_frames = 0;
        if (!analysis.finish_detected && !finish_suppressed &&
            sensor->normal_line_frames < UINT8_MAX) {
            ++sensor->normal_line_frames;
        }
    } else {
        if (sensor->missing_line_frames < UINT8_MAX) {
            ++sensor->missing_line_frames;
        }
        if (sensor->missing_line_frames >= sensor->config.history_arm_frames) {
            sensor->normal_line_frames = 0;
            sensor->finish_candidate_frames = 0;
        }
    }
    /* Seed history only after the normal connected line has been stable long
     * enough to arm ordinary following.  Wide finish-like regions do not
     * overwrite it, including while finish recognition is suppressed. */
    if (reliable_history_candidate &&
        sensor->normal_line_frames >= sensor->config.history_arm_frames) {
        sensor->history_valid = true;
        sensor->history_center_permille = analysis.center_permille;
        sensor->history_steering_permille = analysis.steering_permille;
    }
    sensor->snapshot.virtual_sensors = analysis.virtual_sensors;
    sensor->snapshot.frame_valid = analysis.valid;
    sensor->snapshot.line_detected = analysis.line_detected;
    sensor->snapshot.finish_detected = analysis.finish_detected;
    sensor->snapshot.center_permille = analysis.center_permille;
    sensor->snapshot.far_center_permille = analysis.far_center_permille;
    sensor->snapshot.heading_permille = analysis.heading_permille;
    sensor->snapshot.steering_permille = analysis.steering_permille;
    sensor->snapshot.width_permille = analysis.width_permille;
    sensor->snapshot.component_height_permille =
        analysis.component_height_permille;
    sensor->snapshot.component_area_permille =
        analysis.component_area_permille;
    sensor->snapshot.black_permille = analysis.black_permille;
    sensor->snapshot.connected_component_count =
        analysis.connected_component_count;
    sensor->snapshot.threshold = analysis.threshold;
    sensor->snapshot.contrast = analysis.contrast;
    red_ball = confirm_target(sensor, red_ball, &sensor->red_tracker);
    green_ball = confirm_target(
        sensor, green_ball, &sensor->green_tracker);
    left_target = confirm_target(
        sensor, left_target, &sensor->left_target_tracker);
    right_target = confirm_target(
        sensor, right_target, &sensor->right_target_tracker);
    sensor->snapshot.ball = red_ball;
    sensor->snapshot.green_ball = green_ball;
    sensor->snapshot.left_target = left_target;
    sensor->snapshot.right_target = right_target;
    sensor->snapshot.updated_us = now_us;
    ++sensor->snapshot.decoded_frames;
    portEXIT_CRITICAL(&sensor->lock);
}

static void decode_task(void *argument)
{
    camera_line_sensor_t *sensor = argument;
    const camera_stream_profile_t *profile = active_profile(sensor);
    if (profile == NULL) {
        ESP_LOGE(TAG, "no JPEG decode profile for negotiated stream");
        update_streaming(sensor, false);
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(sensor->frame_queue, &frame, portMAX_DELAY) !=
            pdPASS) {
            continue;
        }

        esp_jpeg_image_cfg_t jpeg_config = {
            .indata = frame->data,
            .indata_size = frame->data_len,
            .outbuf = sensor->rgb_buffer,
            .outbuf_size = sensor->rgb_buffer_size,
            .out_format = JPEG_IMAGE_FORMAT_RGB888,
            .out_scale = profile->decode_scale,
        };
        esp_jpeg_image_output_t output = {0};
        const esp_err_t decode_result = esp_jpeg_decode(&jpeg_config, &output);
        if (decode_result == ESP_OK &&
            output.width <= CAMERA_LINE_OUTPUT_WIDTH &&
            output.height <= CAMERA_LINE_OUTPUT_HEIGHT) {
            bool has_previous_line = false;
            int previous_center_permille = 0;
            int previous_steering_permille = 0;
            portENTER_CRITICAL(&sensor->lock);
            has_previous_line = sensor->history_valid;
            previous_center_permille = sensor->history_center_permille;
            previous_steering_permille =
                sensor->history_steering_permille;
            portEXIT_CRITICAL(&sensor->lock);
            const camera_line_analysis_t analysis =
                camera_line_analyze_rgb888_with_hint(
                    sensor->rgb_buffer, output.width, output.height, false,
                    &sensor->config, sensor->vision_workspace,
                    has_previous_line, previous_center_permille,
                    previous_steering_permille);
            camera_ball_observation_t red_ball =
                camera_ball_analyze_color_rgb888(
                    sensor->rgb_buffer, output.width, output.height, false,
                    BALL_COLOR_RED,
                    &sensor->ball_config, sensor->ball_workspace);
            camera_ball_observation_t green_ball =
                camera_ball_analyze_color_rgb888(
                    sensor->rgb_buffer, output.width, output.height, false,
                    BALL_COLOR_GREEN,
                    &sensor->ball_config, sensor->ball_workspace);
            camera_blue_target_pair_t blue_targets =
                camera_blue_targets_analyze_rgb888(
                    sensor->rgb_buffer, output.width, output.height, false,
                    &sensor->ball_config, sensor->ball_workspace);
            bool dump_view = false;
            portENTER_CRITICAL(&sensor->lock);
            dump_view = sensor->ascii_view_requested;
            sensor->ascii_view_requested = false;
            portEXIT_CRITICAL(&sensor->lock);
            const int64_t decoded_us = esp_timer_get_time();
            if (analysis.valid) {
                publish_analysis(sensor, analysis, red_ball, green_ball,
                                 blue_targets.left_target,
                                 blue_targets.right_target, decoded_us);
                portENTER_CRITICAL(&sensor->lock);
                red_ball = sensor->snapshot.ball;
                green_ball = sensor->snapshot.green_ball;
                blue_targets.left_target = sensor->snapshot.left_target;
                blue_targets.right_target = sensor->snapshot.right_target;
                portEXIT_CRITICAL(&sensor->lock);
            } else {
                portENTER_CRITICAL(&sensor->lock);
                ++sensor->snapshot.decode_errors;
                portEXIT_CRITICAL(&sensor->lock);
            }
            if (dump_view) {
                dump_ascii_view(sensor->rgb_buffer, output.width,
                                output.height);
            }
            bool publish_preview = false;
            uint32_t preview_sequence = 0;
            portENTER_CRITICAL(&sensor->lock);
            if (sensor->usb_preview_enabled && sensor->preview_sink != NULL &&
                sensor->preview_packet != NULL &&
                decoded_us >= sensor->next_preview_us) {
                sensor->next_preview_us = decoded_us +
                    CAMERA_PREVIEW_PERIOD_US;
                preview_sequence = sensor->snapshot.received_frames;
                publish_preview = true;
            }
            portEXIT_CRITICAL(&sensor->lock);
            if (publish_preview && camera_preview_build_rgb332(
                    sensor->preview_packet, sensor->rgb_buffer,
                    output.width, output.height, &sensor->config,
                    &analysis,
                    red_ball.candidate ? &red_ball : &green_ball,
                    &blue_targets.left_target,
                    &blue_targets.right_target, preview_sequence,
                    (uint32_t)(decoded_us / 1000))) {
                sensor->preview_sink(sensor->preview_sink_context,
                                     sensor->preview_packet);
            }
        } else {
            portENTER_CRITICAL(&sensor->lock);
            ++sensor->snapshot.decode_errors;
            portEXIT_CRITICAL(&sensor->lock);
        }

        if (uvc_host_frame_return(sensor->stream, frame) != ESP_OK) {
            portENTER_CRITICAL(&sensor->lock);
            ++sensor->snapshot.decode_errors;
            portEXIT_CRITICAL(&sensor->lock);
        }
        /* The camera can refill the one-frame queue while JPEG decoding. Give
         * the idle task one scheduler tick before taking the next frame. */
        vTaskDelay(1);
    }
}

static esp_err_t open_camera_stream(camera_line_sensor_t *sensor)
{
    for (size_t index = 0;
         index < sizeof(CAMERA_PROFILES) / sizeof(CAMERA_PROFILES[0]);
         ++index) {
        const camera_stream_profile_t *profile = &CAMERA_PROFILES[index];
        const uvc_host_stream_config_t stream_config = {
            .event_cb = camera_stream_event_callback,
            .frame_cb = camera_frame_callback,
            .user_ctx = sensor,
            .usb = {
                .vid = CAMERA_USB_VID,
                .pid = CAMERA_USB_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = profile->width,
                .v_res = profile->height,
                .fps = profile->fps,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = 3,
                .frame_size = CAMERA_FRAME_BUFFER_BYTES,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
                .number_of_urbs = 3,
                .urb_size = CAMERA_URB_BYTES,
                .user_frame_buffers = NULL,
            },
        };
        ESP_LOGI(TAG, "opening %s", profile->name);
        const esp_err_t result = uvc_host_stream_open(
            &stream_config, pdMS_TO_TICKS(5000), &sensor->stream);
        if (result == ESP_OK) {
            sensor->stream_width = profile->width;
            sensor->stream_height = profile->height;
            sensor->stream_fps = profile->fps;
            ESP_LOGI(TAG, "camera negotiated %s", profile->name);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "profile rejected: %s (%s)", profile->name,
                 esp_err_to_name(result));
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t camera_line_sensor_init(camera_line_sensor_t *sensor,
                                  const camera_line_config_t *config,
                                  const camera_ball_config_t *ball_config,
                                  camera_preview_sink_t preview_sink,
                                  void *preview_sink_context)
{
    if (sensor == NULL || config == NULL || ball_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;
    sensor->ball_config = *ball_config;
    sensor->preview_sink = preview_sink;
    sensor->preview_sink_context = preview_sink_context;
    sensor->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    sensor->rgb_buffer_size = CAMERA_LINE_OUTPUT_WIDTH *
                              CAMERA_LINE_OUTPUT_HEIGHT * 3;
    sensor->rgb_buffer = heap_caps_malloc(
        sensor->rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (sensor->rgb_buffer == NULL) return ESP_ERR_NO_MEM;
    sensor->vision_workspace = heap_caps_calloc(
        1, sizeof(*sensor->vision_workspace),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (sensor->vision_workspace == NULL) return ESP_ERR_NO_MEM;
    sensor->ball_workspace = heap_caps_calloc(
        1, sizeof(*sensor->ball_workspace),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (sensor->ball_workspace == NULL) return ESP_ERR_NO_MEM;
    if (preview_sink != NULL) {
        sensor->preview_packet = heap_caps_calloc(
            1, sizeof(*sensor->preview_packet),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (sensor->preview_packet == NULL) return ESP_ERR_NO_MEM;
    }

    sensor->frame_queue = xQueueCreateStatic(
        1, sizeof(uvc_host_frame_t *), sensor->frame_queue_storage,
        &sensor->frame_queue_static);
    if (sensor->frame_queue == NULL) return ESP_ERR_NO_MEM;

    const usb_host_config_t usb_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        .peripheral_map = BIT0,
    };
    esp_err_t result = usb_host_install(&usb_config);
    if (result != ESP_OK) return result;

    sensor->usb_task = xTaskCreateStatic(
        usb_host_event_task, "usb_host_events", CAMERA_LINE_USB_TASK_STACK,
        sensor, 15, sensor->usb_task_stack, &sensor->usb_task_buffer);
    if (sensor->usb_task == NULL) return ESP_ERR_NO_MEM;

    const uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = 16,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = NULL,
        .user_ctx = sensor,
    };
    result = uvc_host_install(&uvc_config);
    if (result != ESP_OK) return result;
    result = open_camera_stream(sensor);
    if (result != ESP_OK) return result;

    sensor->decode_task = xTaskCreateStatic(
        decode_task, "camera_line_decode", CAMERA_LINE_DECODE_TASK_STACK,
        sensor, CAMERA_LINE_DECODE_TASK_PRIORITY, sensor->decode_task_stack,
        &sensor->decode_task_buffer);
    if (sensor->decode_task == NULL) return ESP_ERR_NO_MEM;

    result = uvc_host_stream_start(sensor->stream);
    if (result != ESP_OK) return result;
    update_streaming(sensor, true);
    sensor->initialized = true;
    ESP_LOGI(TAG,
             "camera line sensor started; decode=%dx%d, full-frame "
             "ball and lower-half line analysis, native orientation",
             CAMERA_LINE_OUTPUT_WIDTH, CAMERA_LINE_OUTPUT_HEIGHT);
    return ESP_OK;
}

camera_line_snapshot_t camera_line_sensor_snapshot(
    camera_line_sensor_t *sensor, int64_t now_us)
{
    camera_line_snapshot_t snapshot = {0};
    if (sensor == NULL || !sensor->initialized) return snapshot;
    portENTER_CRITICAL(&sensor->lock);
    snapshot = sensor->snapshot;
    portEXIT_CRITICAL(&sensor->lock);
    snapshot.fresh = snapshot.streaming && snapshot.frame_valid &&
        snapshot.updated_us > 0 &&
        now_us - snapshot.updated_us <= sensor->config.fresh_ms * 1000LL;
    return snapshot;
}

bool camera_line_sensor_request_ascii_view(camera_line_sensor_t *sensor)
{
    if (sensor == NULL || !sensor->initialized) return false;
    bool accepted = false;
    portENTER_CRITICAL(&sensor->lock);
    if (!sensor->ascii_view_requested) {
        sensor->ascii_view_requested = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&sensor->lock);
    return accepted;
}

void camera_line_sensor_set_usb_preview(camera_line_sensor_t *sensor,
                                        bool enabled)
{
    if (sensor == NULL) return;
    portENTER_CRITICAL(&sensor->lock);
    sensor->usb_preview_enabled = enabled;
    if (enabled) sensor->next_preview_us = 0;
    portEXIT_CRITICAL(&sensor->lock);
}

void camera_line_sensor_set_finish_detection_enabled(
    camera_line_sensor_t *sensor, bool enabled)
{
    if (sensor == NULL) return;
    portENTER_CRITICAL(&sensor->lock);
    if (sensor->finish_detection_enabled != enabled) {
        sensor->finish_detection_enabled = enabled;
        sensor->finish_candidate_frames = 0;
        if (!enabled && sensor->snapshot.finish_detected) {
            sensor->snapshot.finish_detected = false;
            sensor->snapshot.virtual_sensors = camera_line_virtual_sensors(
                sensor->snapshot.steering_permille, false);
        }
    }
    portEXIT_CRITICAL(&sensor->lock);
}
