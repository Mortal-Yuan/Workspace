#include "kiwi_kinematics.h"

#include <stdint.h>
#include <stdlib.h>

enum {
    MOTOR_COMMAND_LIMIT = 1000,
};

static int32_t maximum_magnitude(int32_t a, int32_t b, int32_t c)
{
    int32_t maximum = abs((int)a);
    if (abs((int)b) > maximum) maximum = abs((int)b);
    if (abs((int)c) > maximum) maximum = abs((int)c);
    return maximum;
}

motor_command_t kiwi_inverse_kinematics(
    body_motion_command_t motion,
    const kiwi_kinematics_config_t *config)
{
    const int32_t lateral_side =
        (int32_t)motion.left * config->lateral_side_permille / 1000;
    const int32_t compensated_clockwise = (int32_t)motion.clockwise +
        (int32_t)motion.left *
            config->lateral_yaw_compensation_permille / 1000;
    int32_t a = -(int32_t)motion.forward - lateral_side +
                compensated_clockwise;
    int32_t b = -(int32_t)motion.left - compensated_clockwise;
    int32_t c = -(int32_t)motion.forward + lateral_side -
                compensated_clockwise;
    const int32_t maximum = maximum_magnitude(a, b, c);
    if (maximum > MOTOR_COMMAND_LIMIT) {
        a = a * MOTOR_COMMAND_LIMIT / maximum;
        b = b * MOTOR_COMMAND_LIMIT / maximum;
        c = c * MOTOR_COMMAND_LIMIT / maximum;
    }
    if (b > 0 && b < config->motor_b_positive_minimum) {
        b = config->motor_b_positive_minimum;
    }
    if (motion.left != 0 && motion.forward == 0 &&
        motion.clockwise == 0) {
        if (a > 0 && a < config->lateral_side_wheel_minimum) {
            a = config->lateral_side_wheel_minimum;
        } else if (a < 0 && -a < config->lateral_side_wheel_minimum) {
            a = -config->lateral_side_wheel_minimum;
        }
        if (c > 0 && c < config->lateral_side_wheel_minimum) {
            c = config->lateral_side_wheel_minimum;
        } else if (c < 0 && -c < config->lateral_side_wheel_minimum) {
            c = -config->lateral_side_wheel_minimum;
        }
    }
    return (motor_command_t) {
        .a = (int16_t)a,
        .b = (int16_t)b,
        .c = (int16_t)c,
    };
}
