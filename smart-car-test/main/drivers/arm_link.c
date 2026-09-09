#include "arm_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"

enum {
    ARM_LINK_BAUD = 115200,
    ARM_LINK_RX_BUFFER = 256,
    ARM_LINK_TX_BUFFER = 256,
};

static const uart_port_t ARM_LINK_UART = UART_NUM_1;

static arm_link_event_kind_t parse_kind(const char *line)
{
    if (strncmp(line, "ARM,PONG,", 9) == 0) return ARM_LINK_EVENT_PONG;
    if (strncmp(line, "ARM,ACK,", 8) == 0) return ARM_LINK_EVENT_ACK;
    if (strncmp(line, "ARM,BUSY,", 9) == 0) return ARM_LINK_EVENT_BUSY;
    if (strncmp(line, "ARM,DONE,", 9) == 0) return ARM_LINK_EVENT_DONE;
    if (strncmp(line, "ARM,ERROR,", 10) == 0) return ARM_LINK_EVENT_ERROR;
    return ARM_LINK_EVENT_UNKNOWN;
}

static uint16_t parse_sequence(const char *line)
{
    const char *separator = strchr(line, ',');
    if (separator != NULL) separator = strchr(separator + 1, ',');
    if (separator == NULL || separator[1] == '\0') return 0;
    const unsigned long value = strtoul(separator + 1, NULL, 10);
    return value <= UINT16_MAX ? (uint16_t)value : 0;
}

esp_err_t arm_link_init(arm_link_t *link, int tx_pin, int rx_pin)
{
    if (link == NULL) return ESP_ERR_INVALID_ARG;
    memset(link, 0, sizeof(*link));

    const uart_config_t config = {
        .baud_rate = ARM_LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t result = uart_param_config(ARM_LINK_UART, &config);
    if (result != ESP_OK) return result;
    result = uart_set_pin(ARM_LINK_UART, tx_pin, rx_pin,
                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result != ESP_OK) return result;
    result = uart_driver_install(ARM_LINK_UART, ARM_LINK_RX_BUFFER,
                                 ARM_LINK_TX_BUFFER, 0, NULL, 0);
    if (result != ESP_OK) return result;

    link->next_sequence = 1;
    link->initialized = true;
    return ESP_OK;
}

esp_err_t arm_link_send_ping(arm_link_t *link, uint16_t *sequence)
{
    if (link == NULL || !link->initialized) return ESP_ERR_INVALID_STATE;
    const uint16_t current = link->next_sequence++;
    if (link->next_sequence == 0) link->next_sequence = 1;

    char frame[32];
    const int length = snprintf(frame, sizeof(frame),
                                "CAR,PING,%04u\n", current);
    if (length <= 0 || length >= (int)sizeof(frame)) {
        return ESP_ERR_INVALID_SIZE;
    }
    const int written = uart_write_bytes(ARM_LINK_UART, frame, length);
    if (written != length) return ESP_FAIL;
    if (sequence != NULL) *sequence = current;
    return ESP_OK;
}

bool arm_link_poll(arm_link_t *link, arm_link_event_t *event)
{
    if (link == NULL || event == NULL || !link->initialized) return false;
    *event = (arm_link_event_t) {0};

    for (int index = 0; index < 32; ++index) {
        uint8_t raw = 0;
        if (uart_read_bytes(ARM_LINK_UART, &raw, 1, 0) != 1) break;
        const char byte = (char)raw;
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (link->receive_length == 0) continue;
            link->receive_line[link->receive_length] = '\0';
            strncpy(event->line, link->receive_line, sizeof(event->line) - 1);
            event->kind = parse_kind(event->line);
            event->sequence = parse_sequence(event->line);
            link->receive_length = 0;
            return true;
        }
        if (link->receive_length + 1 < sizeof(link->receive_line)) {
            link->receive_line[link->receive_length++] = byte;
        } else {
            link->receive_length = 0;
        }
    }
    return false;
}
