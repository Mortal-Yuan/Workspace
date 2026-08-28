#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#define CAMERA_AP_SSID "SmartCar-Camera"
#define CAMERA_AP_PASSWORD "smartcar123"
#define CAMERA_AP_CHANNEL 6
#define CAMERA_AP_MAX_CONNECTIONS 2
#define CAMERA_WEB_ADDRESS "http://192.168.4.1/"

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

typedef struct {
    unsigned width;
    unsigned height;
    float fps;
    const char *name;
} stream_profile_t;

static const stream_profile_t STREAM_PROFILES[] = {
    {640, 480, 15.0f, "MJPEG 640x480 @ 15 fps"},
    {320, 240, 30.0f, "MJPEG 320x240 @ 30 fps"},
    {320, 240, 0.0f, "MJPEG 320x240 @ camera-default fps"},
};

static const char *TAG = "camera_preview";
static QueueHandle_t s_frame_queue;
static SemaphoreHandle_t s_stream_client_lock;
static uvc_host_stream_hdl_t s_uvc_stream;
static httpd_handle_t s_http_server;
static volatile bool s_camera_streaming;
static volatile uint32_t s_frame_count;
static volatile size_t s_total_frame_bytes;
static volatile uint32_t s_dropped_frame_count;
static volatile uint32_t s_http_client_count;
static volatile unsigned s_stream_width;
static volatile unsigned s_stream_height;
static volatile float s_stream_fps;

static const char ROOT_PAGE[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Smart Car Camera</title><style>"
    "body{margin:0;background:#111;color:#eee;font-family:system-ui;text-align:center}"
    "main{max-width:900px;margin:auto;padding:16px}"
    "img{width:100%;height:auto;background:#222;border-radius:10px}"
    "#status{margin:10px;color:#9fd}small{color:#aaa}"
    "</style></head><body><main><h2>Smart Car Camera</h2>"
    "<div id='status'>Connecting...</div><img src='/stream' alt='camera stream'>"
    "<p><small>Camera preview uses its native orientation. Motor outputs are locked off.</small></p>"
    "</main><script>setInterval(async()=>{try{let r=await fetch('/status',"
    "{cache:'no-store'});let s=await r.json();document.getElementById('status')."
    "textContent=`${s.width}x${s.height} @ ${s.fps} fps | frames ${s.frames} | "
    "dropped ${s.dropped}`;}catch(e){}},1000);</script></body></html>";

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

static void wifi_event_callback(
    void *argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)argument;
    (void)event_base;
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event =
            (const wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG,
                 "viewer connected: " MACSTR " aid=%d",
                 MAC2STR(event->mac),
                 event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event =
            (const wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG,
                 "viewer disconnected: " MACSTR " aid=%d reason=%d",
                 MAC2STR(event->mac),
                 event->aid,
                 event->reason);
    }
}

static void initialize_camera_access_point(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_ap() != NULL
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_callback,
        NULL,
        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CAMERA_AP_SSID,
            .password = CAMERA_AP_PASSWORD,
            .ssid_len = sizeof(CAMERA_AP_SSID) - 1,
            .channel = CAMERA_AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = CAMERA_AP_MAX_CONNECTIONS,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Wi-Fi AP ready: SSID=%s", CAMERA_AP_SSID);
    ESP_LOGI(TAG, "Wi-Fi password: %s", CAMERA_AP_PASSWORD);
    ESP_LOGI(TAG, "preview address: %s", CAMERA_WEB_ADDRESS);
}

static bool camera_frame_callback(
    const uvc_host_frame_t *frame, void *user_context)
{
    (void)user_context;
    if (frame == NULL || frame->data == NULL || frame->data_len == 0) {
        return true;
    }

    ++s_frame_count;
    s_total_frame_bytes += frame->data_len;
    uvc_host_frame_t *queued_frame = (uvc_host_frame_t *)frame;
    if (xQueueSendToBack(s_frame_queue, &queued_frame, 0) != pdPASS) {
        ++s_dropped_frame_count;
        return true;
    }
    return false;
}

static void camera_stream_event_callback(
    const uvc_host_stream_event_data_t *event, void *user_context)
{
    (void)user_context;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG,
                 "UVC transfer error: %s",
                 esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        s_camera_streaming = false;
        ESP_LOGE(TAG, "camera disconnected");
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "camera frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ++s_dropped_frame_count;
        break;
    default:
        ESP_LOGW(TAG, "unhandled UVC event %d", event->type);
        break;
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
    }
}

static esp_err_t open_camera_stream(void)
{
    for (size_t profile_index = 0;
         profile_index < sizeof(STREAM_PROFILES) / sizeof(STREAM_PROFILES[0]);
         ++profile_index) {
        const stream_profile_t *profile = &STREAM_PROFILES[profile_index];
        const uvc_host_stream_config_t stream_config = {
            .event_cb = camera_stream_event_callback,
            .frame_cb = camera_frame_callback,
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
                .number_of_frame_buffers = 3,
                .frame_size = 0,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
                .number_of_urbs = 3,
                .urb_size = 10 * 1024,
                .user_frame_buffers = NULL,
            },
        };

        ESP_LOGI(TAG, "opening %s", profile->name);
        esp_err_t error = uvc_host_stream_open(
            &stream_config, pdMS_TO_TICKS(5000), &s_uvc_stream);
        if (error == ESP_OK) {
            s_stream_width = profile->width;
            s_stream_height = profile->height;
            s_stream_fps = profile->fps;
            ESP_LOGI(TAG, "camera negotiated %s", profile->name);
            return ESP_OK;
        }
        ESP_LOGW(TAG,
                 "camera profile rejected: %s (%s)",
                 profile->name,
                 esp_err_to_name(error));
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t root_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, ROOT_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    char response[240];
    int length = snprintf(
        response,
        sizeof(response),
        "{\"camera\":\"%s\",\"width\":%u,\"height\":%u,"
        "\"fps\":%.1f,\"frames\":%u,\"bytes\":%u,\"dropped\":%u,"
        "\"viewers\":%u}",
        s_camera_streaming ? "streaming" : "waiting",
        s_stream_width,
        s_stream_height,
        (double)s_stream_fps,
        (unsigned)s_frame_count,
        (unsigned)s_total_frame_bytes,
        (unsigned)s_dropped_frame_count,
        (unsigned)s_http_client_count);
    if (length < 0 || (size_t)length >= sizeof(response)) {
        return httpd_resp_send_500(request);
    }
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

static esp_err_t stream_handler(httpd_req_t *request)
{
    if (xSemaphoreTake(s_stream_client_lock, 0) != pdTRUE) {
        httpd_resp_set_status(request, "503 Busy");
        return httpd_resp_sendstr(request, "A preview client is already connected.");
    }

    ++s_http_client_count;
    ESP_LOGI(TAG, "browser MJPEG stream opened");
    httpd_resp_set_type(
        request, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");

    esp_err_t result = ESP_OK;
    while (result == ESP_OK) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(
                s_frame_queue, &frame, pdMS_TO_TICKS(5000)) != pdPASS) {
            if (!s_camera_streaming) {
                result = ESP_FAIL;
            }
            continue;
        }

        char part_header[128];
        int header_length = snprintf(
            part_header,
            sizeof(part_header),
            "--frame\r\nContent-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",
            (unsigned)frame->data_len);
        if (header_length < 0 ||
            (size_t)header_length >= sizeof(part_header)) {
            result = ESP_FAIL;
        } else {
            result = httpd_resp_send_chunk(
                request, part_header, header_length);
        }
        if (result == ESP_OK) {
            result = httpd_resp_send_chunk(
                request, (const char *)frame->data, frame->data_len);
        }
        if (result == ESP_OK) {
            result = httpd_resp_send_chunk(request, "\r\n", 2);
        }

        esp_err_t return_error = uvc_host_frame_return(s_uvc_stream, frame);
        if (return_error != ESP_OK) {
            ESP_LOGE(TAG,
                     "could not return UVC frame: %s",
                     esp_err_to_name(return_error));
            result = return_error;
        }
    }

    --s_http_client_count;
    ESP_LOGI(TAG, "browser MJPEG stream closed");
    xSemaphoreGive(s_stream_client_lock);
    return result;
}

static void start_http_preview_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    ESP_ERROR_CHECK(httpd_start(&s_http_server, &config));

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_page_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &stream_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &status_uri));
}

void app_main(void)
{
    force_motors_safe();
    ESP_LOGI(TAG, "motors locked off; IR and motion control are disabled");
    ESP_LOGI(TAG,
             "PSRAM total=%u bytes free=%u bytes",
             (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    s_frame_queue = xQueueCreate(2, sizeof(uvc_host_frame_t *));
    s_stream_client_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_frame_queue != NULL && s_stream_client_lock != NULL
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);

    initialize_camera_access_point();
    start_http_preview_server();

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
        .event_cb = NULL,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));
    ESP_ERROR_CHECK(open_camera_stream());
    ESP_ERROR_CHECK(uvc_host_stream_start(s_uvc_stream));
    s_camera_streaming = true;

    ESP_LOGI(TAG, "CAMERA_PREVIEW_RESULT=READY");
    ESP_LOGI(TAG, "connect Wi-Fi %s and open %s", CAMERA_AP_SSID, CAMERA_WEB_ADDRESS);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG,
                 "preview running: frames=%u dropped=%u viewers=%u",
                 (unsigned)s_frame_count,
                 (unsigned)s_dropped_frame_count,
                 (unsigned)s_http_client_count);
    }
}
