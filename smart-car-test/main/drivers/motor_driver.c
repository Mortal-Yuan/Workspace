#include "motor_driver.h"

#include <stdlib.h>
#include <string.h>

static int clamp_command(int value)
{
    return value < -1000 ? -1000 : value > 1000 ? 1000 : value;
}

static int sign_of(int value)
{
    return value < 0 ? -1 : value > 0 ? 1 : 0;
}

static int command_at(motor_command_t command, int channel)
{
    return channel == 0 ? command.a : channel == 1 ? command.b : command.c;
}

static uint32_t command_duty(const motor_driver_t *driver, int command)
{
    return (uint32_t)abs(command) * driver->hal.pwm_max_duty / 1000U;
}

static bool emergency_disable(motor_driver_t *driver)
{
    driver->hardware_write_count++;
    const bool disabled = motor_hal_set_enable(&driver->hal, false);
    driver->enabled = false;
    driver->fault_latched = true;
    driver->recoverable = disabled;
    return disabled;
}

motor_result_t motor_driver_preinit_safe(motor_driver_t *driver,
                                         const motor_hal_config_t *config)
{
    if (driver == NULL || config == NULL) {
        return MOTOR_RESULT_INVALID_COMMAND;
    }
    memset(driver, 0, sizeof(*driver));
    driver->hal = *config;
    driver->hardware_write_count++;
    if (!motor_hal_preinit_safe(&driver->hal)) {
        driver->fault_latched = true;
        driver->recoverable = false;
        return MOTOR_RESULT_UNRECOVERABLE_FAULT;
    }
    return MOTOR_RESULT_OK;
}

motor_result_t motor_driver_init(motor_driver_t *driver)
{
    if (driver == NULL || driver->fault_latched) {
        return MOTOR_RESULT_FAULT_LATCHED;
    }
    driver->hardware_write_count++;
    if (!motor_hal_init_outputs(&driver->hal)) {
        return emergency_disable(driver) ? MOTOR_RESULT_RECOVERABLE_FAULT :
                                           MOTOR_RESULT_UNRECOVERABLE_FAULT;
    }
    driver->cached = motor_command_zero();
    driver->initialized = true;
    return MOTOR_RESULT_OK;
}

motor_result_t motor_driver_apply(motor_driver_t *driver,
                                  motor_command_t command)
{
    if (driver == NULL || !driver->initialized) {
        return MOTOR_RESULT_UNRECOVERABLE_FAULT;
    }
    if (driver->fault_latched) {
        return MOTOR_RESULT_FAULT_LATCHED;
    }
    command.a = (int16_t)clamp_command(command.a);
    command.b = (int16_t)clamp_command(command.b);
    command.c = (int16_t)clamp_command(command.c);
    if (memcmp(&command, &driver->cached, sizeof(command)) == 0) {
        return MOTOR_RESULT_OK;
    }

    const bool target_active = !motor_command_is_zero(command);
    if (!target_active) {
        driver->hardware_write_count++;
        if (!motor_hal_set_enable(&driver->hal, false)) {
            emergency_disable(driver);
            driver->recoverable = false;
            return MOTOR_RESULT_UNRECOVERABLE_FAULT;
        }
        driver->enabled = false;
    }

    for (int channel = 0; channel < 3; ++channel) {
        const int old_command = command_at(driver->cached, channel);
        const int new_command = command_at(command, channel);
        const int old_direction = sign_of(old_command);
        const int new_direction = sign_of(new_command);
        const bool direction_change = old_direction != new_direction;
        bool duty_cleared_for_direction = false;

        if (old_direction != 0 && direction_change) {
            driver->hardware_write_count++;
            if (!motor_hal_set_duty(&driver->hal, channel, 0)) {
                emergency_disable(driver);
                return MOTOR_RESULT_RECOVERABLE_FAULT;
            }
            duty_cleared_for_direction = true;
        }
        if (direction_change) {
            driver->hardware_write_count++;
            if (!motor_hal_set_direction(&driver->hal, channel,
                                         new_direction)) {
                emergency_disable(driver);
                return MOTOR_RESULT_RECOVERABLE_FAULT;
            }
        }
        if (old_command != new_command &&
            !(new_command == 0 && duty_cleared_for_direction)) {
            driver->hardware_write_count++;
            if (!motor_hal_set_duty(&driver->hal, channel,
                                    command_duty(driver, new_command))) {
                emergency_disable(driver);
                return MOTOR_RESULT_RECOVERABLE_FAULT;
            }
        }
    }

    if (target_active && !driver->enabled) {
        driver->hardware_write_count++;
        if (!motor_hal_set_enable(&driver->hal, true)) {
            emergency_disable(driver);
            return MOTOR_RESULT_RECOVERABLE_FAULT;
        }
        driver->enabled = true;
    }
    driver->cached = command;
    return MOTOR_RESULT_OK;
}

motor_result_t motor_driver_safe_reinit(motor_driver_t *driver)
{
    if (driver == NULL || !driver->recoverable) {
        return MOTOR_RESULT_UNRECOVERABLE_FAULT;
    }
    driver->hardware_write_count++;
    if (!motor_hal_set_enable(&driver->hal, false)) {
        driver->recoverable = false;
        return MOTOR_RESULT_UNRECOVERABLE_FAULT;
    }
    if (!motor_hal_init_outputs(&driver->hal)) {
        return MOTOR_RESULT_RECOVERABLE_FAULT;
    }
    driver->cached = motor_command_zero();
    driver->enabled = false;
    driver->initialized = true;
    driver->fault_latched = false;
    return MOTOR_RESULT_OK;
}

motor_command_t motor_driver_command(const motor_driver_t *driver)
{
    return driver != NULL ? driver->cached : motor_command_zero();
}

bool motor_driver_enabled(const motor_driver_t *driver)
{
    return driver != NULL && driver->enabled;
}
