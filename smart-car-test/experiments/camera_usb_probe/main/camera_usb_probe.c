#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_helpers.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

/*
 * Standalone, read-only USB camera probe.
 *
 * This firmware deliberately does not initialize the smart-car application,
 * line sensors, PWM, or motion-control tasks.  Motor enable and every motor
 * input are forced low before USB Host is started.
 */

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

typedef enum {
    PROBE_ACTION_NONE = 0,
    PROBE_ACTION_OPEN = 1 << 0,
    PROBE_ACTION_CLOSE = 1 << 1,
} probe_action_t;

typedef struct {
    usb_host_client_handle_t client;
    usb_device_handle_t device;
    uint8_t device_address;
    uint8_t actions;
    bool report_complete;
} probe_state_t;

static const char *TAG = "camera_probe";
static probe_state_t s_probe;

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

static const char *speed_name(usb_speed_t speed)
{
    switch (speed) {
    case USB_SPEED_LOW:
        return "low-speed";
    case USB_SPEED_FULL:
        return "full-speed";
    case USB_SPEED_HIGH:
        return "high-speed";
    default:
        return "unknown-speed";
    }
}

static void summarize_interfaces(const usb_config_desc_t *config_desc)
{
    unsigned video_interfaces = 0;
    unsigned audio_interfaces = 0;
    unsigned other_interfaces = 0;
    int offset = 0;
    const usb_standard_desc_t *descriptor =
        (const usb_standard_desc_t *)config_desc;

    while ((descriptor = usb_parse_next_descriptor(
                descriptor, config_desc->wTotalLength, &offset)) != NULL) {
        if (descriptor->bDescriptorType != USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            continue;
        }

        const usb_intf_desc_t *interface =
            (const usb_intf_desc_t *)descriptor;
        ESP_LOGI(TAG,
                 "interface=%u alt=%u class=0x%02x subclass=0x%02x "
                 "protocol=0x%02x endpoints=%u",
                 interface->bInterfaceNumber,
                 interface->bAlternateSetting,
                 interface->bInterfaceClass,
                 interface->bInterfaceSubClass,
                 interface->bInterfaceProtocol,
                 interface->bNumEndpoints);

        /* Count each interface once; alternate settings share its number. */
        if (interface->bAlternateSetting != 0) {
            continue;
        }
        if (interface->bInterfaceClass == USB_CLASS_VIDEO) {
            ++video_interfaces;
        } else if (interface->bInterfaceClass == USB_CLASS_AUDIO) {
            ++audio_interfaces;
        } else {
            ++other_interfaces;
        }
    }

    ESP_LOGI(TAG,
             "interface summary: video=%u audio=%u other=%u",
             video_interfaces,
             audio_interfaces,
             other_interfaces);
    if (video_interfaces > 0) {
        ESP_LOGI(TAG,
                 "CAMERA_RESULT=PASS_UVC camera interface detected%s",
                 audio_interfaces > 0 ? " (composite audio also detected)" : "");
    } else {
        ESP_LOGE(TAG, "CAMERA_RESULT=FAIL_NO_UVC_INTERFACE");
    }
}

static esp_err_t open_and_report_device(void)
{
    esp_err_t error = usb_host_device_open(
        s_probe.client, s_probe.device_address, &s_probe.device);
    if (error != ESP_OK) {
        ESP_LOGE(TAG,
                 "cannot open USB address %u: %s",
                 s_probe.device_address,
                 esp_err_to_name(error));
        return error;
    }

    usb_device_info_t device_info;
    error = usb_host_device_info(s_probe.device, &device_info);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "cannot read device info: %s", esp_err_to_name(error));
        return error;
    }

    const usb_device_desc_t *device_desc = NULL;
    error = usb_host_get_device_descriptor(s_probe.device, &device_desc);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "cannot read device descriptor: %s", esp_err_to_name(error));
        return error;
    }

    ESP_LOGI(TAG,
             "USB device opened: address=%u speed=%s configuration=%u",
             device_info.dev_addr,
             speed_name(device_info.speed),
             device_info.bConfigurationValue);
    ESP_LOGI(TAG,
             "VID=0x%04x PID=0x%04x USB=%x.%02x device=%x.%02x class=0x%02x",
             device_desc->idVendor,
             device_desc->idProduct,
             device_desc->bcdUSB >> 8,
             device_desc->bcdUSB & 0xff,
             device_desc->bcdDevice >> 8,
             device_desc->bcdDevice & 0xff,
             device_desc->bDeviceClass);
    usb_print_device_descriptor(device_desc);

    if (device_info.str_desc_manufacturer != NULL) {
        ESP_LOGI(TAG, "manufacturer string:");
        usb_print_string_descriptor(device_info.str_desc_manufacturer);
    }
    if (device_info.str_desc_product != NULL) {
        ESP_LOGI(TAG, "product string:");
        usb_print_string_descriptor(device_info.str_desc_product);
    }
    if (device_info.str_desc_serial_num != NULL) {
        ESP_LOGI(TAG, "serial-number string:");
        usb_print_string_descriptor(device_info.str_desc_serial_num);
    }

    const usb_config_desc_t *config_desc = NULL;
    error = usb_host_get_active_config_descriptor(s_probe.device, &config_desc);
    if (error != ESP_OK) {
        ESP_LOGE(TAG,
                 "cannot read active configuration descriptor: %s",
                 esp_err_to_name(error));
        return error;
    }

    ESP_LOGI(TAG,
             "configuration descriptor: total_length=%u interfaces=%u",
             config_desc->wTotalLength,
             config_desc->bNumInterfaces);
    usb_print_config_descriptor(config_desc, NULL);
    summarize_interfaces(config_desc);
    s_probe.report_complete = true;
    return ESP_OK;
}

static void close_device(void)
{
    if (s_probe.device == NULL) {
        return;
    }
    esp_err_t error = usb_host_device_close(s_probe.client, s_probe.device);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "cannot close USB device: %s", esp_err_to_name(error));
    }
    s_probe.device = NULL;
    s_probe.device_address = 0;
    s_probe.report_complete = false;
    ESP_LOGW(TAG, "camera disconnected; waiting for reconnection");
}

static void client_event_callback(
    const usb_host_client_event_msg_t *event_message, void *argument)
{
    probe_state_t *probe = (probe_state_t *)argument;

    switch (event_message->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        if (probe->device == NULL) {
            probe->device_address = event_message->new_dev.address;
            probe->actions |= PROBE_ACTION_OPEN;
            ESP_LOGI(TAG,
                     "new USB device at address %u",
                     event_message->new_dev.address);
        } else {
            ESP_LOGW(TAG, "additional USB device ignored by single-device probe");
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        if (probe->device == event_message->dev_gone.dev_hdl) {
            probe->actions = PROBE_ACTION_CLOSE;
        }
        break;
    default:
        ESP_LOGW(TAG, "unhandled USB client event %d", event_message->event);
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
        }
    }
}

void app_main(void)
{
    force_motors_safe();
    ESP_LOGI(TAG, "motors locked off; IR line-following firmware is not running");
    ESP_LOGI(TAG, "starting ESP32-S3 internal USB Host on GPIO19(D-) / GPIO20(D+)");

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .peripheral_map = BIT0,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    BaseType_t task_created = xTaskCreatePinnedToCore(
        usb_host_event_task, "usb_host_events", 4096, NULL, 2, NULL, 0);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_callback,
            .callback_arg = &s_probe,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &s_probe.client));

    ESP_LOGI(TAG, "camera probe ready; waiting for a USB device");
    TickType_t last_wait_message = xTaskGetTickCount();
    while (true) {
        esp_err_t error = usb_host_client_handle_events(
            s_probe.client, pdMS_TO_TICKS(500));
        if (error != ESP_OK && error != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "USB client event error: %s", esp_err_to_name(error));
        }

        uint8_t actions = s_probe.actions;
        s_probe.actions = PROBE_ACTION_NONE;
        if ((actions & PROBE_ACTION_CLOSE) != 0) {
            close_device();
        }
        if ((actions & PROBE_ACTION_OPEN) != 0) {
            error = open_and_report_device();
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "CAMERA_RESULT=FAIL_DESCRIPTOR_READ");
                close_device();
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (!s_probe.report_complete &&
            now - last_wait_message >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "still waiting for camera enumeration...");
            last_wait_message = now;
        }
    }
}
