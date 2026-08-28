#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_COMMAND_BATCH_MAX 16

typedef struct {
    char bytes[APP_COMMAND_BATCH_MAX];
    uint8_t count;
} command_batch_t;

typedef enum {
    APP_MODE_IDLE,
    APP_MODE_AUTONOMOUS,
    APP_MODE_MANUAL,
    APP_MODE_SELF_TEST,
    APP_MODE_FAULT,
} app_mode_t;

typedef enum {
    FAULT_SOURCE_MOTOR,
    FAULT_SOURCE_PLATFORM_ISR,
    FAULT_SOURCE_CAMERA,
    FAULT_SOURCE_ULTRASONIC,
    FAULT_SOURCE_START_BUTTON,
    FAULT_SOURCE_CONTROL_CONFIG,
    FAULT_SOURCE_COUNT,
} fault_source_t;

typedef enum {
    FAULT_RECOVERABLE_EXPLICIT_REINIT,
    FAULT_UNRECOVERABLE_THIS_BOOT,
} fault_recoverability_t;

typedef struct {
    fault_source_t source;
    int32_t code;
    fault_recoverability_t recoverability;
    bool active;
} fault_record_t;
