#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int enable_pin;
    int pwm_pin[3];
    int in1_pin[3];
    int in2_pin[3];
    int pwm_channel[3];
    int enable_level;
    uint32_t pwm_max_duty;
    uint32_t pwm_frequency_hz;
} motor_hal_config_t;

bool motor_hal_preinit_safe(const motor_hal_config_t *config);
bool motor_hal_init_outputs(const motor_hal_config_t *config);
bool motor_hal_set_enable(const motor_hal_config_t *config, bool enabled);
bool motor_hal_set_direction(const motor_hal_config_t *config,
                             int channel, int direction);
bool motor_hal_set_duty(const motor_hal_config_t *config,
                        int channel, uint32_t duty);
