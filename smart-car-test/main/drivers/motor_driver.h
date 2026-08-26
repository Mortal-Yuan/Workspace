#pragma once

#include <stdbool.h>

#include "motion_types.h"
#include "motor_hal.h"

typedef enum {
    MOTOR_RESULT_OK,
    MOTOR_RESULT_INVALID_COMMAND,
    MOTOR_RESULT_RECOVERABLE_FAULT,
    MOTOR_RESULT_UNRECOVERABLE_FAULT,
    MOTOR_RESULT_FAULT_LATCHED,
} motor_result_t;

typedef struct {
    motor_hal_config_t hal;
    motor_command_t cached;
    bool initialized;
    bool enabled;
    bool fault_latched;
    bool recoverable;
    uint32_t hardware_write_count;
} motor_driver_t;

motor_result_t motor_driver_preinit_safe(motor_driver_t *driver,
                                         const motor_hal_config_t *config);
motor_result_t motor_driver_init(motor_driver_t *driver);
motor_result_t motor_driver_apply(motor_driver_t *driver,
                                  motor_command_t command);
motor_result_t motor_driver_safe_reinit(motor_driver_t *driver);
motor_command_t motor_driver_command(const motor_driver_t *driver);
bool motor_driver_enabled(const motor_driver_t *driver);
