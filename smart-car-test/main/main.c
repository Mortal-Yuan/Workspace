#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_START_BUTTON      GPIO_NUM_0
#define PIN_IR_RIGHT          GPIO_NUM_1
#define PIN_IR_RIGHT_CENTER   GPIO_NUM_2
#define PIN_IR_LEFT_CENTER    GPIO_NUM_4
#define PIN_IR_LEFT           GPIO_NUM_5
#define PIN_US_TRIG           GPIO_NUM_6
#define PIN_MOTOR_ENABLE      GPIO_NUM_7
#define PIN_ENCODER_A_B       GPIO_NUM_8
#define PIN_MOTOR_A_PWM       GPIO_NUM_9
#define PIN_MOTOR_A_IN1       GPIO_NUM_10
#define PIN_MOTOR_A_IN2       GPIO_NUM_11
#define PIN_MOTOR_B_PWM       GPIO_NUM_12
#define PIN_US_ECHO           GPIO_NUM_13
#define PIN_MOTOR_B_IN1       GPIO_NUM_14
#define PIN_MOTOR_B_IN2       GPIO_NUM_15
#define PIN_MOTOR_C_PWM       GPIO_NUM_16
#define PIN_MOTOR_C_IN1       GPIO_NUM_17
#define PIN_ENCODER_A_A       GPIO_NUM_18
#define PIN_MOTOR_C_IN2       GPIO_NUM_21
#define PIN_ENCODER_B_A       GPIO_NUM_39
#define PIN_ENCODER_B_B       GPIO_NUM_40
#define PIN_ENCODER_C_A       GPIO_NUM_41
#define PIN_ENCODER_C_B       GPIO_NUM_42

#define MOTOR_ENABLE_LEVEL    1
#define PWM_FREQUENCY_HZ      20000
#define PWM_MAX_DUTY          1023
#define DEFAULT_SPEED         400
#define SHORT_TEST_SPEED      400
#define LINE_FOLLOW_PERIOD_MS 20
#define LINE_STRAIGHT_SPEED   320
#define LINE_CURVE_SPEED      220
#define LINE_CURVE_MAX_CMD    360
#define LINE_FOLLOW_KP        90
#define LINE_MAX_CORRECTION   220
#define LINE_SEARCH_SPEED     280
#define LINE_EDGE_SPEED       170
#define LINE_EDGE_MAX_CMD     280
#define LINE_DIRECTION_CONFIRM_COUNT 3
#define LINE_DIRECTION_HOLD_ERROR    4
#define LINE_MONITOR_PERIOD_MS 100
#define START_BUTTON_DEBOUNCE_MS 300
#define TELEMETRY_PERIOD_MS   500
#define ULTRASONIC_TIMEOUT_US 30000
#define ULTRASONIC_PERIOD_MS  60
#define OBSTACLE_STOP_MM      250
#define OBSTACLE_CLEAR_MM     350
#define OBSTACLE_CONFIRM_COUNT 2
#define OBSTACLE_CLEAR_COUNT  2
#define OBSTACLE_BRAKE_MS     250
#define OBSTACLE_TURN_MIN_MS  850
#define OBSTACLE_TURN_MAX_MS  2400
#define OBSTACLE_TURN_SPEED   190

typedef struct {
    bool left;
    bool left_center;
    bool right_center;
    bool right;
} line_sensors_t;

typedef enum {
    LINE_STATE_IDLE,
    LINE_STATE_STRAIGHT,
    LINE_STATE_CURVE,
    LINE_STATE_SEARCH_LEFT,
    LINE_STATE_SEARCH_RIGHT,
} line_state_t;

typedef enum {
    OBSTACLE_STATE_CLEAR,
    OBSTACLE_STATE_BRAKE,
    OBSTACLE_STATE_TURN_RIGHT,
} obstacle_state_t;

static volatile int64_t s_encoder_a_count;
static volatile int64_t s_encoder_b_count;
static volatile int64_t s_encoder_c_count;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_ultrasonic_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_speed = DEFAULT_SPEED;
static bool s_line_follow_enabled;
static bool s_line_monitor_enabled = true;
static line_sensors_t s_last_line_sensors;
static line_state_t s_line_state = LINE_STATE_IDLE;
static int s_line_error;
static int s_line_control_error;
static int s_last_line_error;
static int s_locked_direction;
static int s_direction_candidate;
static unsigned int s_direction_candidate_count;
static int s_line_active_count;
static unsigned int s_line_lost_count;
static uint8_t s_line_pattern;
static uint8_t s_previous_line_pattern;
static unsigned int s_line_pattern_stable_count;
static int s_line_base_speed;
static int s_motor_a_command;
static int s_motor_b_command;
static int s_motor_c_command;
static volatile int64_t s_ultrasonic_echo_start_us;
static volatile int64_t s_ultrasonic_pulse_us;
static volatile bool s_ultrasonic_pulse_ready;
static volatile bool s_ultrasonic_capture_started;
static bool s_ultrasonic_waiting;
static int64_t s_ultrasonic_trigger_us;
static int64_t s_next_ultrasonic_us;
static int64_t s_ultrasonic_update_us;
static int s_ultrasonic_distance_mm = -1;
static unsigned int s_ultrasonic_timeout_count;
static obstacle_state_t s_obstacle_state = OBSTACLE_STATE_CLEAR;
static int64_t s_obstacle_state_start_us;
static int64_t s_obstacle_last_sample_us;
static unsigned int s_obstacle_near_count;
static unsigned int s_obstacle_clear_count;

static void encoder_a_isr(void *arg)
{
    const int a = gpio_get_level(PIN_ENCODER_A_A);
    const int b = gpio_get_level(PIN_ENCODER_A_B);

    portENTER_CRITICAL_ISR(&s_encoder_lock);
    s_encoder_a_count += (a == b) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

static void encoder_b_isr(void *arg)
{
    const int a = gpio_get_level(PIN_ENCODER_B_A);
    const int b = gpio_get_level(PIN_ENCODER_B_B);

    portENTER_CRITICAL_ISR(&s_encoder_lock);
    s_encoder_b_count += (a == b) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

static void encoder_c_isr(void *arg)
{
    const int a = gpio_get_level(PIN_ENCODER_C_A);
    const int b = gpio_get_level(PIN_ENCODER_C_B);

    portENTER_CRITICAL_ISR(&s_encoder_lock);
    s_encoder_c_count += (a == b) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

static void ultrasonic_echo_isr(void *arg)
{
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&s_ultrasonic_lock);
    if (!s_ultrasonic_capture_started) {
        s_ultrasonic_echo_start_us = now_us;
        s_ultrasonic_capture_started = true;
    } else {
        s_ultrasonic_pulse_us = now_us - s_ultrasonic_echo_start_us;
        s_ultrasonic_pulse_ready = true;
        s_ultrasonic_capture_started = false;
    }
    portEXIT_CRITICAL_ISR(&s_ultrasonic_lock);
}

static void configure_gpio(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_ENABLE) |
                        (1ULL << PIN_MOTOR_A_IN1) |
                        (1ULL << PIN_MOTOR_A_IN2) |
                        (1ULL << PIN_MOTOR_B_IN1) |
                        (1ULL << PIN_MOTOR_B_IN2) |
                        (1ULL << PIN_MOTOR_C_IN1) |
                        (1ULL << PIN_MOTOR_C_IN2) |
                        (1ULL << PIN_US_TRIG),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));

    gpio_set_level(PIN_MOTOR_ENABLE, !MOTOR_ENABLE_LEVEL);
    gpio_set_level(PIN_MOTOR_A_IN1, 0);
    gpio_set_level(PIN_MOTOR_A_IN2, 0);
    gpio_set_level(PIN_MOTOR_B_IN1, 0);
    gpio_set_level(PIN_MOTOR_B_IN2, 0);
    gpio_set_level(PIN_MOTOR_C_IN1, 0);
    gpio_set_level(PIN_MOTOR_C_IN2, 0);
    gpio_set_level(PIN_US_TRIG, 0);

    const gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << PIN_IR_LEFT) |
                        (1ULL << PIN_IR_LEFT_CENTER) |
                        (1ULL << PIN_IR_RIGHT_CENTER) |
                        (1ULL << PIN_IR_RIGHT) |
                        (1ULL << PIN_US_ECHO) |
                        (1ULL << PIN_ENCODER_A_A) |
                        (1ULL << PIN_ENCODER_A_B) |
                        (1ULL << PIN_ENCODER_B_A) |
                        (1ULL << PIN_ENCODER_B_B) |
                        (1ULL << PIN_ENCODER_C_A) |
                        (1ULL << PIN_ENCODER_C_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_config));

    const gpio_config_t start_button_config = {
        .pin_bit_mask = 1ULL << PIN_START_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&start_button_config));

    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_ENCODER_A_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_ENCODER_B_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_ENCODER_C_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_US_ECHO, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_A_A, encoder_a_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_B_A, encoder_b_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_C_A, encoder_c_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_US_ECHO,
                                         ultrasonic_echo_isr, NULL));
}

static void configure_pwm(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t motor_a_channel = {
        .gpio_num = PIN_MOTOR_A_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_a_channel));

    const ledc_channel_config_t motor_b_channel = {
        .gpio_num = PIN_MOTOR_B_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_b_channel));

    const ledc_channel_config_t motor_c_channel = {
        .gpio_num = PIN_MOTOR_C_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_c_channel));
}

static int clamp_value(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void set_pwm(ledc_channel_t channel, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void set_one_motor(ledc_channel_t channel, gpio_num_t in1_pin,
                          gpio_num_t in2_pin, int speed)
{
    if (speed > 1000) {
        speed = 1000;
    } else if (speed < -1000) {
        speed = -1000;
    }

    set_pwm(channel, 0);
    if (speed == 0) {
        gpio_set_level(in1_pin, 0);
        gpio_set_level(in2_pin, 0);
        return;
    }

    gpio_set_level(in1_pin, speed > 0 ? 1 : 0);
    gpio_set_level(in2_pin, speed > 0 ? 0 : 1);
    const uint32_t magnitude = (uint32_t)(speed > 0 ? speed : -speed);
    set_pwm(channel, magnitude * PWM_MAX_DUTY / 1000U);
}

static void set_motors(int motor_a_speed, int motor_b_speed, int motor_c_speed)
{
    motor_a_speed = clamp_value(motor_a_speed, -1000, 1000);
    motor_b_speed = clamp_value(motor_b_speed, -1000, 1000);
    motor_c_speed = clamp_value(motor_c_speed, -1000, 1000);
    s_motor_a_command = motor_a_speed;
    s_motor_b_command = motor_b_speed;
    s_motor_c_command = motor_c_speed;

    set_one_motor(LEDC_CHANNEL_0, PIN_MOTOR_A_IN1,
                  PIN_MOTOR_A_IN2, motor_a_speed);
    set_one_motor(LEDC_CHANNEL_1, PIN_MOTOR_B_IN1,
                  PIN_MOTOR_B_IN2, motor_b_speed);
    set_one_motor(LEDC_CHANNEL_2, PIN_MOTOR_C_IN1,
                  PIN_MOTOR_C_IN2, motor_c_speed);
}

static line_sensors_t read_line_sensors(void)
{
    return (line_sensors_t) {
        .left = gpio_get_level(PIN_IR_LEFT) == 0,
        .left_center = gpio_get_level(PIN_IR_LEFT_CENTER) == 0,
        .right_center = gpio_get_level(PIN_IR_RIGHT_CENTER) == 0,
        .right = gpio_get_level(PIN_IR_RIGHT) == 0,
    };
}

static uint8_t line_sensor_pattern(line_sensors_t sensors)
{
    return (uint8_t)((sensors.left << 3) |
                     (sensors.left_center << 2) |
                     (sensors.right_center << 1) |
                     sensors.right);
}

static bool is_straight_pattern(uint8_t pattern)
{
    return pattern == 0x06 || pattern == 0x0f;
}

static bool is_side_only_pattern(uint8_t pattern)
{
    const bool left_side = (pattern & 0x0c) != 0;
    const bool right_side = (pattern & 0x03) != 0;
    return left_side != right_side;
}

static int sign_of(int value)
{
    return value < 0 ? -1 : value > 0 ? 1 : 0;
}

static void update_direction_lock(int direction)
{
    if (direction == 0) {
        return;
    }

    if (s_locked_direction == 0) {
        if (s_direction_candidate != direction) {
            s_direction_candidate = direction;
            s_direction_candidate_count = 1;
        } else if (s_direction_candidate_count <
                   LINE_DIRECTION_CONFIRM_COUNT) {
            s_direction_candidate_count++;
        }

        if (s_direction_candidate_count >=
            LINE_DIRECTION_CONFIRM_COUNT) {
            s_locked_direction = direction;
            s_direction_candidate = 0;
            s_direction_candidate_count = 0;
        }
        return;
    }

    if (direction == s_locked_direction) {
        s_direction_candidate = 0;
        s_direction_candidate_count = 0;
        return;
    }

    if (s_direction_candidate != direction) {
        s_direction_candidate = direction;
        s_direction_candidate_count = 1;
    } else if (s_direction_candidate_count < LINE_DIRECTION_CONFIRM_COUNT) {
        s_direction_candidate_count++;
    }

    if (s_direction_candidate_count >= LINE_DIRECTION_CONFIRM_COUNT) {
        s_locked_direction = direction;
        s_direction_candidate = 0;
        s_direction_candidate_count = 0;
    }
}

static void set_curve_motors(int motor_a_speed, int motor_c_speed,
                             int command_limit)
{
    int maximum = motor_a_speed < 0 ? -motor_a_speed : motor_a_speed;
    const int motor_c_magnitude =
        motor_c_speed < 0 ? -motor_c_speed : motor_c_speed;
    if (motor_c_magnitude > maximum) {
        maximum = motor_c_magnitude;
    }

    if (maximum > command_limit) {
        motor_a_speed = motor_a_speed * command_limit / maximum;
        motor_c_speed = motor_c_speed * command_limit / maximum;
    }
    set_motors(motor_a_speed, 0, motor_c_speed);
}

static void set_search_motors(int direction)
{
    if (direction < 0) {
        s_line_state = LINE_STATE_SEARCH_LEFT;
        set_motors(-LINE_SEARCH_SPEED, 0, LINE_SEARCH_SPEED);
    } else {
        s_line_state = LINE_STATE_SEARCH_RIGHT;
        set_motors(LINE_SEARCH_SPEED, 0, -LINE_SEARCH_SPEED);
    }
}

static void update_lost_line_recovery(void)
{
    s_line_lost_count++;
    s_line_error = 0;
    s_line_control_error = 0;
    s_line_base_speed = 0;
    set_search_motors(s_locked_direction == 0 ? -1 : s_locked_direction);
}

static void update_line_following(void)
{
    const line_sensors_t sensors = read_line_sensors();
    const uint8_t pattern = line_sensor_pattern(sensors);
    const int active_count = sensors.left + sensors.left_center +
                             sensors.right_center + sensors.right;
    s_last_line_sensors = sensors;
    s_line_active_count = active_count;
    s_line_pattern = pattern;

    if (pattern == s_previous_line_pattern) {
        if (s_line_pattern_stable_count < 255) {
            s_line_pattern_stable_count++;
        }
    } else {
        s_previous_line_pattern = pattern;
        s_line_pattern_stable_count = 1;
    }

    if (active_count == 0) {
        update_lost_line_recovery();
        return;
    }

    s_line_lost_count = 0;

    const int weighted_sum = sensors.left * -6 +
                             sensors.left_center * -2 +
                             sensors.right_center * 2 +
                             sensors.right * 6;
    const int line_error = weighted_sum / active_count;
    const int line_direction = sign_of(line_error);
    update_direction_lock(line_direction);
    s_line_error = line_error;
    if (line_error != 0) {
        s_last_line_error = line_error;
    }

    const bool stable_straight = is_straight_pattern(pattern);
    const bool side_only = is_side_only_pattern(pattern);
    int base_speed = stable_straight ? LINE_STRAIGHT_SPEED :
                     side_only ? LINE_EDGE_SPEED : LINE_CURVE_SPEED;
    if (base_speed > s_speed) {
        base_speed = s_speed;
    }
    s_line_state = stable_straight ? LINE_STATE_STRAIGHT : LINE_STATE_CURVE;
    s_line_base_speed = base_speed;
    int control_error = line_error;
    const bool opposite_unconfirmed =
        s_locked_direction != 0 && line_direction != 0 &&
        line_direction != s_locked_direction &&
        s_direction_candidate_count < LINE_DIRECTION_CONFIRM_COUNT;
    if (opposite_unconfirmed) {
        control_error = s_locked_direction *
                        (abs(line_error) > LINE_DIRECTION_HOLD_ERROR ?
                         abs(line_error) : LINE_DIRECTION_HOLD_ERROR);
    }
    s_line_control_error = control_error;
    const int correction = clamp_value(control_error * LINE_FOLLOW_KP,
                                       -LINE_MAX_CORRECTION,
                                       LINE_MAX_CORRECTION);

    if (stable_straight) {
        set_motors(-base_speed + correction, 0,
                   -base_speed - correction);
    } else {
        const int curve_limit = side_only ? LINE_EDGE_MAX_CMD :
                                            LINE_CURVE_MAX_CMD;
        set_curve_motors(-base_speed + correction,
                         -base_speed - correction,
                         curve_limit);
    }
}

static void disable_line_following(void)
{
    s_line_follow_enabled = false;
    s_line_lost_count = 0;
    s_line_base_speed = 0;
    s_line_control_error = 0;
    s_line_state = LINE_STATE_IDLE;
    s_locked_direction = 0;
    s_direction_candidate = 0;
    s_direction_candidate_count = 0;
    s_obstacle_state = OBSTACLE_STATE_CLEAR;
    s_obstacle_near_count = 0;
    s_obstacle_clear_count = 0;
}

static void ultrasonic_trigger(int64_t now_us)
{
    portENTER_CRITICAL(&s_ultrasonic_lock);
    s_ultrasonic_echo_start_us = 0;
    s_ultrasonic_pulse_ready = false;
    s_ultrasonic_capture_started = false;
    portEXIT_CRITICAL(&s_ultrasonic_lock);

    gpio_set_level(PIN_US_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(PIN_US_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_US_TRIG, 0);

    s_ultrasonic_waiting = true;
    s_ultrasonic_trigger_us = now_us;
    s_next_ultrasonic_us = now_us + ULTRASONIC_PERIOD_MS * 1000LL;
}

static void update_ultrasonic(int64_t now_us)
{
    int64_t pulse_us = 0;
    bool pulse_ready;

    portENTER_CRITICAL(&s_ultrasonic_lock);
    pulse_ready = s_ultrasonic_pulse_ready;
    if (pulse_ready) {
        pulse_us = s_ultrasonic_pulse_us;
        s_ultrasonic_pulse_ready = false;
    }
    portEXIT_CRITICAL(&s_ultrasonic_lock);

    if (pulse_ready) {
        const int distance_mm = (int)(pulse_us * 343 / 2000);
        s_ultrasonic_waiting = false;
        if (distance_mm >= 20 && distance_mm <= 4000) {
            s_ultrasonic_distance_mm = distance_mm;
            s_ultrasonic_update_us = now_us;
            s_ultrasonic_timeout_count = 0;
        }
    } else if (s_ultrasonic_waiting &&
               now_us - s_ultrasonic_trigger_us > ULTRASONIC_TIMEOUT_US) {
        s_ultrasonic_waiting = false;
        s_ultrasonic_timeout_count++;
        if (s_ultrasonic_timeout_count >= 3) {
            s_ultrasonic_distance_mm = -1;
        }
    }

    if (!s_ultrasonic_waiting && now_us >= s_next_ultrasonic_us) {
        ultrasonic_trigger(now_us);
    }
}

static const char *obstacle_state_name(obstacle_state_t state)
{
    switch (state) {
    case OBSTACLE_STATE_CLEAR:
        return "CLEAR";
    case OBSTACLE_STATE_BRAKE:
        return "BRAKE";
    case OBSTACLE_STATE_TURN_RIGHT:
        return "TURN_RIGHT";
    default:
        return "UNKNOWN";
    }
}

static bool update_obstacle_avoidance(int64_t now_us)
{
    const bool distance_fresh = s_ultrasonic_distance_mm >= 0 &&
        now_us - s_ultrasonic_update_us <= 200000;
    const bool new_sample = distance_fresh &&
        s_ultrasonic_update_us != s_obstacle_last_sample_us;
    if (new_sample) {
        s_obstacle_last_sample_us = s_ultrasonic_update_us;
    }

    if (s_obstacle_state == OBSTACLE_STATE_CLEAR) {
        if (new_sample &&
            s_ultrasonic_distance_mm <= OBSTACLE_STOP_MM) {
            s_obstacle_near_count++;
        } else if (new_sample) {
            s_obstacle_near_count = 0;
        }

        if (s_obstacle_near_count < OBSTACLE_CONFIRM_COUNT) {
            return false;
        }

        s_obstacle_state = OBSTACLE_STATE_BRAKE;
        s_obstacle_state_start_us = now_us;
        s_obstacle_near_count = 0;
        s_obstacle_clear_count = 0;
        set_motors(0, 0, 0);
        printf("OBSTACLE detected distance=%d mm; braking\n",
               s_ultrasonic_distance_mm);
        return true;
    }

    if (s_obstacle_state == OBSTACLE_STATE_BRAKE) {
        set_motors(0, 0, 0);
        if (now_us - s_obstacle_state_start_us >=
            OBSTACLE_BRAKE_MS * 1000LL) {
            s_obstacle_state = OBSTACLE_STATE_TURN_RIGHT;
            s_obstacle_state_start_us = now_us;
            printf("OBSTACLE turning right to find a clear path\n");
        }
        return true;
    }

    set_motors(OBSTACLE_TURN_SPEED, 0, -OBSTACLE_TURN_SPEED);
    const int64_t turn_time_us = now_us - s_obstacle_state_start_us;
    if (turn_time_us >= OBSTACLE_TURN_MIN_MS * 1000LL) {
        if (new_sample &&
            s_ultrasonic_distance_mm >= OBSTACLE_CLEAR_MM) {
            s_obstacle_clear_count++;
        } else if (new_sample) {
            s_obstacle_clear_count = 0;
        }

        if (s_obstacle_clear_count >= OBSTACLE_CLEAR_COUNT ||
            turn_time_us >= OBSTACLE_TURN_MAX_MS * 1000LL) {
            printf("OBSTACLE cleared distance=%d mm; resuming line search\n",
                   s_ultrasonic_distance_mm);
            s_obstacle_state = OBSTACLE_STATE_CLEAR;
            s_obstacle_near_count = 0;
            s_obstacle_clear_count = 0;
            s_line_lost_count = 0;
            return false;
        }
    }
    return true;
}

static void read_encoder_counts(int64_t *motor_a, int64_t *motor_b,
                                int64_t *motor_c)
{
    portENTER_CRITICAL(&s_encoder_lock);
    *motor_a = s_encoder_a_count;
    *motor_b = s_encoder_b_count;
    *motor_c = s_encoder_c_count;
    portEXIT_CRITICAL(&s_encoder_lock);
}

static const char *line_state_name(line_state_t state)
{
    switch (state) {
    case LINE_STATE_IDLE:
        return "IDLE";
    case LINE_STATE_STRAIGHT:
        return "STRAIGHT";
    case LINE_STATE_CURVE:
        return "CURVE";
    case LINE_STATE_SEARCH_LEFT:
        return "SEARCH_LEFT";
    case LINE_STATE_SEARCH_RIGHT:
        return "SEARCH_RIGHT";
    default:
        return "UNKNOWN";
    }
}

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  BOOT   start infrared line following\n");
    printf("  w/s  forward/backward\n");
    printf("  f    start infrared line following\n");
    printf("  m    toggle 10 Hz line-follow monitor\n");
    printf("  a/d  uncalibrated manual turn test\n");
    printf("  1/2/3  A-right/B-rear/C-left motor only\n");
    printf("  r      short A/B/C motor test\n");
    printf("  x    stop\n");
    printf("  +/-  change speed (current %d/1000)\n", s_speed);
    printf("  c    clear encoder counters\n");
    printf("  h    show this help\n\n");
}

static void run_short_test(void)
{
    printf("Test: motor A (right wheel)\n");
    set_motors(SHORT_TEST_SPEED, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_motors(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("Test: motor B (rear wheel)\n");
    set_motors(0, SHORT_TEST_SPEED, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_motors(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("Test: motor C (left wheel)\n");
    set_motors(0, 0, SHORT_TEST_SPEED);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_motors(0, 0, 0);
    printf("Test complete\n");
}

static void enable_line_following(const char *source)
{
    set_motors(0, 0, 0);
    s_last_line_sensors = read_line_sensors();
    s_line_state = LINE_STATE_IDLE;
    s_line_error = 0;
    s_line_control_error = 0;
    s_last_line_error = 0;
    s_locked_direction = 0;
    s_direction_candidate = 0;
    s_direction_candidate_count = 0;
    s_line_active_count = 0;
    s_line_lost_count = 0;
    s_line_pattern = line_sensor_pattern(s_last_line_sensors);
    s_previous_line_pattern = s_line_pattern;
    s_line_pattern_stable_count = 0;
    s_line_base_speed = 0;
    s_obstacle_state = OBSTACLE_STATE_CLEAR;
    s_obstacle_last_sample_us = s_ultrasonic_update_us;
    s_obstacle_near_count = 0;
    s_obstacle_clear_count = 0;
    s_line_follow_enabled = true;
    printf("Line following + obstacle avoidance enabled by %s; "
           "monitor=%s; press x to stop\n",
           source, s_line_monitor_enabled ? "ON" : "OFF");
    fflush(stdout);
}

static void handle_command(char command)
{
    switch (tolower((unsigned char)command)) {
    case 'w':
        disable_line_following();
        set_motors(-s_speed, 0, -s_speed);
        printf("Forward: A=-%d B=0 C=-%d\n", s_speed, s_speed);
        break;
    case 's':
        disable_line_following();
        set_motors(s_speed, 0, s_speed);
        printf("Backward: A=%d B=0 C=%d\n", s_speed, s_speed);
        break;
    case 'a':
        disable_line_following();
        set_motors(-s_speed, s_speed, 0);
        printf("Rotate left\n");
        break;
    case 'd':
        disable_line_following();
        set_motors(s_speed, -s_speed, 0);
        printf("Rotate right\n");
        break;
    case '1':
        disable_line_following();
        set_motors(s_speed, 0, 0);
        printf("Motor A only (right wheel)\n");
        break;
    case '2':
        disable_line_following();
        set_motors(0, s_speed, 0);
        printf("Motor B only (rear wheel)\n");
        break;
    case '3':
        disable_line_following();
        set_motors(0, 0, s_speed);
        printf("Motor C only (left wheel)\n");
        break;
    case 'r':
        disable_line_following();
        run_short_test();
        break;
    case 'f':
        enable_line_following("serial f");
        break;
    case 'm':
        s_line_monitor_enabled = !s_line_monitor_enabled;
        printf("Line monitor: %s\n",
               s_line_monitor_enabled ? "ON (10 Hz)" : "OFF");
        break;
    case 'x':
        disable_line_following();
        set_motors(0, 0, 0);
        printf("Stop\n");
        break;
    case '+':
        s_speed += 50;
        if (s_speed > 800) {
            s_speed = 800;
        }
        printf("Speed: %d/1000\n", s_speed);
        break;
    case '-':
        s_speed -= 50;
        if (s_speed < 100) {
            s_speed = 100;
        }
        printf("Speed: %d/1000\n", s_speed);
        break;
    case 'c':
        portENTER_CRITICAL(&s_encoder_lock);
        s_encoder_a_count = 0;
        s_encoder_b_count = 0;
        s_encoder_c_count = 0;
        portEXIT_CRITICAL(&s_encoder_lock);
        printf("Encoder counters cleared\n");
        break;
    case 'h':
        print_help();
        break;
    case '\r':
    case '\n':
        break;
    default:
        printf("Unknown command '%c'; press h for help\n", command);
        break;
    }
    fflush(stdout);
}

static void print_telemetry(void)
{
    int64_t motor_a_count;
    int64_t motor_b_count;
    int64_t motor_c_count;
    read_encoder_counts(&motor_a_count, &motor_b_count, &motor_c_count);

    const line_sensors_t sensors = read_line_sensors();
    printf("IR[L,LC,RC,R]=%d%d%d%d  mode=%s  distance=",
           sensors.left, sensors.left_center,
           sensors.right_center, sensors.right,
           s_line_follow_enabled ? "FOLLOW" : "MANUAL");
    if (s_ultrasonic_distance_mm >= 0) {
        printf("%d mm", s_ultrasonic_distance_mm);
    } else {
        printf("waiting");
    }
    printf("  encoder A=%" PRId64 " B=%" PRId64 " C=%" PRId64 "\n",
           motor_a_count, motor_b_count, motor_c_count);
    fflush(stdout);
}

static void print_line_monitor(void)
{
    int64_t motor_a_count;
    int64_t motor_b_count;
    int64_t motor_c_count;
    read_encoder_counts(&motor_a_count, &motor_b_count, &motor_c_count);

    printf("FOLLOW t=%" PRId64 "ms state=%s IR[L,LC,RC,R]=%d%d%d%d "
           "pattern=0x%X stable=%u active=%d error=%d control=%d "
           "last=%d lock=%d candidate=%d:%u base=%d "
           "lost=%u distance=%dmm obstacle=%s "
           "cmd[A,B,C]=%d,%d,%d "
           "encoder[A,B,C]=%" PRId64 ",%" PRId64 ",%" PRId64 "\n",
           esp_timer_get_time() / 1000,
           line_state_name(s_line_state),
           s_last_line_sensors.left,
           s_last_line_sensors.left_center,
           s_last_line_sensors.right_center,
           s_last_line_sensors.right,
           s_line_pattern,
           s_line_pattern_stable_count,
           s_line_active_count,
           s_line_error,
           s_line_control_error,
           s_last_line_error,
           s_locked_direction,
           s_direction_candidate,
           s_direction_candidate_count,
           s_line_base_speed,
           s_line_lost_count,
           s_ultrasonic_distance_mm,
           obstacle_state_name(s_obstacle_state),
           s_motor_a_command,
           s_motor_b_command,
           s_motor_c_command,
           motor_a_count,
           motor_b_count,
           motor_c_count);
    fflush(stdout);
}

void app_main(void)
{
    configure_gpio();
    configure_pwm();
    set_motors(0, 0, 0);
    gpio_set_level(PIN_MOTOR_ENABLE, MOTOR_ENABLE_LEVEL);

    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    usb_serial_jtag_vfs_use_driver();

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    const int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK);
    }

    printf("\nSmart car test ready. Motors are stopped.\n");
    print_help();

    int64_t next_telemetry_us = esp_timer_get_time();
    int64_t next_line_follow_us = esp_timer_get_time();
    int64_t next_line_monitor_us = esp_timer_get_time();
    int64_t last_start_button_us = 0;
    bool start_button_was_pressed =
        gpio_get_level(PIN_START_BUTTON) == 0;
    while (true) {
        char command;
        if (read(STDIN_FILENO, &command, 1) == 1) {
            handle_command(command);
        }

        const int64_t now_us = esp_timer_get_time();
        update_ultrasonic(now_us);
        const bool start_button_is_pressed =
            gpio_get_level(PIN_START_BUTTON) == 0;
        if (start_button_is_pressed && !start_button_was_pressed &&
            now_us - last_start_button_us >=
                START_BUTTON_DEBOUNCE_MS * 1000LL) {
            last_start_button_us = now_us;
            if (!s_line_follow_enabled) {
                enable_line_following("BOOT button");
            }
        }
        start_button_was_pressed = start_button_is_pressed;

        if (s_line_follow_enabled && now_us >= next_line_follow_us) {
            if (!update_obstacle_avoidance(now_us)) {
                update_line_following();
            }
            next_line_follow_us = now_us + LINE_FOLLOW_PERIOD_MS * 1000LL;
        }
        if (s_line_follow_enabled && s_line_monitor_enabled &&
            now_us >= next_line_monitor_us) {
            print_line_monitor();
            next_line_monitor_us = now_us + LINE_MONITOR_PERIOD_MS * 1000LL;
        }
        if (!s_line_follow_enabled && now_us >= next_telemetry_us) {
            print_telemetry();
            next_telemetry_us = now_us + TELEMETRY_PERIOD_MS * 1000LL;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
