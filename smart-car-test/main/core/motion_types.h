#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t a;
    int16_t b;
    int16_t c;
} motor_command_t;

/*
 * Chassis-centred command axes.  Positive values mean forward, left and
 * clockwise respectively.  Each component uses the same normalized
 * -1000..1000 command range as the motor driver.
 */
typedef struct {
    int16_t forward;
    int16_t left;
    int16_t clockwise;
} body_motion_command_t;

static inline motor_command_t motor_command_zero(void)
{
    return (motor_command_t) {0, 0, 0};
}

static inline bool motor_command_is_zero(motor_command_t command)
{
    return command.a == 0 && command.b == 0 && command.c == 0;
}

static inline body_motion_command_t body_motion_zero(void)
{
    return (body_motion_command_t) {0, 0, 0};
}
