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
};

typedef struct {
    unsigned width;
    unsigned height;
    float fps;
    esp_jpeg_image_scale_t decode_scale;
    const char *name;
} camera_stream_profile_t;

static const camera_stream_profile_t CAMERA_PROFILES[] = {
    {640, 480, 15.0f, JPEG_IMAGE_SCALE_1_8, "MJPEG 640x480 @ 15 fps"},
};

static const char *TAG = "camera_line";

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
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

static void publish_analysis(camera_line_sensor_t *sensor,
                             camera_line_analysis_t analysis,
                             int64_t now_us)
{
    portENTER_CRITICAL(&sensor->lock);
    const bool reliable_history_candidate =
        analysis.line_detected && !analysis.finish_detected;
    const bool finish_suppressed = analysis.finish_detected &&
        sensor->normal_line_frames < sensor->config.finish_arm_frames;
    if (finish_suppressed) {
        /* A wide dark object at startup is not a finish line.  The finish
         * shape is armed only after a normal connected track was followed.
         * It remains a valid connected line instead of being rejected. */
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
        if (sensor->missing_line_frames >= sensor->config.finish_arm_frames) {
            sensor->normal_line_frames = 0;
        }
    }
    /* Seed history only after the normal connected line has been stable long
     * enough to arm ordinary following.  Wide finish-like regions do not
     * overwrite it, including while finish recognition is suppressed. */
    if (reliable_history_candidate &&
        sensor->normal_line_frames >= sensor->config.finish_arm_frames) {
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
                camera_line_analyze_lower_half_rgb888_with_hint(
                    sensor->rgb_buffer, output.width, output.height, false,
                    &sensor->config, sensor->vision_workspace,
                    has_previous_line, previous_center_permille,
                    previous_steering_permille);
            if (analysis.valid) {
                publish_analysis(sensor, analysis, esp_timer_get_time());
            } else {
                portENTER_CRITICAL(&sensor->lock);
                ++sensor->snapshot.decode_errors;
                portEXIT_CRITICAL(&sensor->lock);
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
                                  const camera_line_config_t *config)
{
    if (sensor == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;
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
        sensor, 8, sensor->decode_task_stack, &sensor->decode_task_buffer);
    if (sensor->decode_task == NULL) return ESP_ERR_NO_MEM;

    result = uvc_host_stream_start(sensor->stream);
    if (result != ESP_OK) return result;
    update_streaming(sensor, true);
    sensor->initialized = true;
    ESP_LOGI(TAG,
             "camera line sensor started; decode=%dx%d lower-half analysis "
             "native orientation",
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
