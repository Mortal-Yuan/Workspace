#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define ARM_LINK_LINE_MAX 64

typedef enum {
    ARM_LINK_EVENT_NONE,
    ARM_LINK_EVENT_PONG,
    ARM_LINK_EVENT_ACK,
    ARM_LINK_EVENT_BUSY,
    ARM_LINK_EVENT_DONE,
    ARM_LINK_EVENT_ERROR,
    ARM_LINK_EVENT_UNKNOWN,
} arm_link_event_kind_t;

typedef struct {
    arm_link_event_kind_t kind;
    uint16_t sequence;
    char line[ARM_LINK_LINE_MAX];
} arm_link_event_t;

typedef struct {
    char receive_line[ARM_LINK_LINE_MAX];
    size_t receive_length;
    uint16_t next_sequence;
    bool initialized;
} arm_link_t;

esp_err_t arm_link_init(arm_link_t *link, int tx_pin, int rx_pin);
esp_err_t arm_link_send_ping(arm_link_t *link, uint16_t *sequence);
bool arm_link_poll(arm_link_t *link, arm_link_event_t *event);
