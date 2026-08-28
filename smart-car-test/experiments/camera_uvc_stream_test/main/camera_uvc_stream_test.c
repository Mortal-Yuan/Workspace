#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

enum {
    PIN_MOTOR_ENABLE = GPIO_NUM_7,
    PIN_MOTOR_A_PWM = GPIO_NUM_9,
    PIN_MOTOR_A_IN1 = GPIO_NUM_10,
    PIN_MOTOR_A_IN2 = GPIO_NUM_11,
    PIN_MOTOR_B_PWM = GPIO_NUM_12,
    PIN_MOTOR_B_IN1 = GPIO_NUM_14,
    PIN_MOTOR_B_IN2 = GPIO_NUM_15,
    PIN_MOTOR_C_PWM = GPIO_NUM_16,
    PIN_MOTOR_C_IN1 = GPIO_NUM_17,
    PIN_MOTOR_C_IN2 = GPIO_NUM_21,
};

#define MOTOR_SAFE_PIN_MASK \
    ((1ULL << PIN_MOTOR_ENABLE) | (1ULL << PIN_MOTOR_A_PWM) | \
     (1ULL << PIN_MOTOR_A_IN1) | (1ULL << PIN_MOTOR_A_IN2) | \
     (1ULL << PIN_MOTOR_B_PWM) | (1ULL << PIN_MOTOR_B_IN1) | \
     (1ULL << PIN_MOTOR_B_IN2) | (1ULL << PIN_MOTOR_C_PWM) | \
     (1ULL << PIN_MOTOR_C_IN1) | (1ULL << PIN_MOTOR_C_IN2))

#define STREAM_PASS_FRAME_COUNT 10U
#define STREAM_TEST_TIMEOUT_MS 15000U
#define STREAM_RESULT_BIT BIT(16)
#define STREAM_DISCONNECTED_BIT BIT(17)
#define STREAM_TRANSFER_ERROR_BIT BIT(18)

typedef struct {
    uint16_t width;
    uint16_t height;
    float fps;
    const char *name;
} stream_profile_t;

static const stream_profile_t STREAM_PROFILES[] = {
    {640, 480, 15.0f, "MJPEG 640x480 @ 15 fps"},
    {320, 240, 30.0f, "MJPEG 320x240 @ 30 fps"},
    {320, 240, 0.0f, "MJPEG 320x240 @ camera-default fps"},
};

static const char *TAG = "camera_stream";
static EventGroupHandle_t s_stream_events;
static volatile uint32_t s_frame_count;
static volatile size_t s_total_bytes;
static volatile size_t s_last_frame_bytes;

static void force_motors_safe(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask = MOTOR_SAFE_PIN_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&output_config));
    for (int pin = 0; pin < GPIO_NUM_MAX; ++pin) {
        if ((MOTOR_SAFE_PIN_MASK & (1ULL << pin)) != 0) {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)pin, 0));
        }
    }
}

static void usb_host_event_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t event_flags = 0;
        esp_err_t error = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "USB Host event error: %s", esp_err_to_name(error));
            continue;
        }
        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0) {
            usb_host_device_free_all();
        }
        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) != 0) {
            ESP_LOGI(TAG, "all USB devices freed");
        }
    }
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_context)
{
    (void)user_context;
    if (frame == NULL || frame->data == NULL || frame->data_len == 0) {
        return true;
    }

    ++s_frame_count;
    s_total_bytes += frame->data_len;
    s_last_frame_bytes = frame->data_len;

    if (s_frame_count == 1 ||
        s_frame_count == 5 ||
        s_frame_count == STREAM_PASS_FRAME_COUNT) {
        ESP_LOGI(TAG,
                 "valid frame %u: %u bytes",
                 (unsigned)s_frame_count,
                 (unsigned)frame->data_len);
    }
    if (s_frame_count >= STREAM_PASS_FRAME_COUNT) {
        xEventGroupSetBits(s_stream_events, STREAM_RESULT_BIT);
    }

    /* No deferred processing in this diagnostic; immediately recycle buffer. */
    return true;
}

static void stream_event_callback(
    const uvc_host_stream_event_data_t *event, void *user_context)
{
    (void)user_context;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG,
                 "UVC transfer error: %d",
                 event->transfer_error.error);
        xEventGroupSetBits(s_stream_events, STREAM_TRANSFER_ERROR_BIT);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGE(TAG, "UVC camera disconnected");
        xEventGroupSetBits(s_stream_events, STREAM_DISCONNECTED_BIT);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "UVC frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "UVC frame buffer underflow");
        break;
    default:
        ESP_LOGW(TAG, "unhandled UVC stream event %d", event->type);
        break;
    }
}

static esp_err_t open_stream_with_fallback(uvc_host_stream_hdl_t *stream)
{
    for (size_t profile_index = 0;
         profile_index < sizeof(STREAM_PROFILES) / sizeof(STREAM_PROFILES[0]);
         ++profile_index) {
        const stream_profile_t *profile = &STREAM_PROFILES[profile_index];
        const uvc_host_stream_config_t stream_config = {
            .event_cb = stream_event_callback,
            .frame_cb = frame_callback,
            .user_ctx = NULL,
            .usb = {
                .vid = 0x349c,
                .pid = 0x3307,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = profile->width,
                .v_res = profile->height,
                .fps = profile->fps,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .frame_size = 0,
                .number_of_frame_buffers = 3,
                .number_of_urbs = 3,
                .urb_size = 10 * 1024,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
            },
        };

        ESP_LOGI(TAG, "opening %s", profile->name);
        esp_err_t error = uvc_host_stream_open(
            &stream_config, pdMS_TO_TICKS(5000), stream);
        if (error == ESP_OK) {
            ESP_LOGI(TAG, "negotiated %s", profile->name);
            return ESP_OK;
        }
        ESP_LOGW(TAG,
                 "profile rejected: %s (%s)",
                 profile->name,
                 esp_err_to_name(error));
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static void stay_safe_forever(void)
{
    ESP_LOGI(TAG, "test finished; motors remain locked off");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    force_motors_safe();
    ESP_LOGI(TAG, "motors locked off; starting native UVC Host stream test");
    ESP_LOGI(TAG,
             "PSRAM total=%u bytes free=%u bytes",
             (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    s_stream_events = xEventGroupCreate();
    ESP_ERROR_CHECK(s_stream_events != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    const usb_host_config_t usb_host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        .peripheral_map = BIT0,
    };
    ESP_ERROR_CHECK(usb_host_install(&usb_host_config));

    BaseType_t task_created = xTaskCreatePinnedToCore(
        usb_host_event_task,
        "usb_host_events",
        4096,
        NULL,
        15,
        NULL,
        tskNO_AFFINITY);
    ESP_ERROR_CHECK(task_created == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = 16,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));

    uvc_host_stream_hdl_t stream = NULL;
    esp_err_t error = open_stream_with_fallback(&stream);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "CAMERA_STREAM_RESULT=FAIL_NEGOTIATION");
        stay_safe_forever();
    }

    error = uvc_host_stream_start(stream);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "stream start failed: %s", esp_err_to_name(error));
        ESP_LOGE(TAG, "CAMERA_STREAM_RESULT=FAIL_START_STREAM");
        uvc_host_stream_close(stream);
        stay_safe_forever();
    }

    ESP_LOGI(TAG,
             "streaming started; waiting for %u valid frames",
             STREAM_PASS_FRAME_COUNT);
    EventBits_t result = xEventGroupWaitBits(
        s_stream_events,
        STREAM_RESULT_BIT | STREAM_DISCONNECTED_BIT | STREAM_TRANSFER_ERROR_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(STREAM_TEST_TIMEOUT_MS));

    if ((result & STREAM_RESULT_BIT) != 0) {
        ESP_LOGI(TAG,
                 "CAMERA_STREAM_RESULT=PASS frames=%u total_bytes=%u "
                 "last_frame=%u",
                 (unsigned)s_frame_count,
                 (unsigned)s_total_bytes,
                 (unsigned)s_last_frame_bytes);
    } else if ((result & STREAM_DISCONNECTED_BIT) != 0) {
        ESP_LOGE(TAG,
                 "CAMERA_STREAM_RESULT=FAIL_DISCONNECTED frames=%u",
                 (unsigned)s_frame_count);
    } else if ((result & STREAM_TRANSFER_ERROR_BIT) != 0) {
        ESP_LOGE(TAG,
                 "CAMERA_STREAM_RESULT=FAIL_TRANSFER frames=%u",
                 (unsigned)s_frame_count);
    } else {
        ESP_LOGE(TAG,
                 "CAMERA_STREAM_RESULT=FAIL_TIMEOUT frames=%u",
                 (unsigned)s_frame_count);
    }

    uvc_host_stream_stop(stream);
    uvc_host_stream_close(stream);
    stay_safe_forever();
}
