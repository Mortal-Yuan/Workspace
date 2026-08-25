#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
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

#define PIN_IR_1              GPIO_NUM_1
#define PIN_IR_2              GPIO_NUM_2
#define PIN_IR_3              GPIO_NUM_4
#define PIN_IR_4              GPIO_NUM_5
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
#define TELEMETRY_PERIOD_MS   500
#define ULTRASONIC_TIMEOUT_US 30000

static volatile int64_t s_encoder_a_count;
static volatile int64_t s_encoder_b_count;
static volatile int64_t s_encoder_c_count;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_speed = DEFAULT_SPEED;

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
        .pin_bit_mask = (1ULL << PIN_IR_1) |
                        (1ULL << PIN_IR_2) |
                        (1ULL << PIN_IR_3) |
                        (1ULL << PIN_IR_4) |
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

    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_ENCODER_A_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_ENCODER_B_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_ENCODER_C_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_A_A, encoder_a_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_B_A, encoder_b_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_C_A, encoder_c_isr, NULL));
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
    set_one_motor(LEDC_CHANNEL_0, PIN_MOTOR_A_IN1,
                  PIN_MOTOR_A_IN2, motor_a_speed);
    set_one_motor(LEDC_CHANNEL_1, PIN_MOTOR_B_IN1,
                  PIN_MOTOR_B_IN2, motor_b_speed);
    set_one_motor(LEDC_CHANNEL_2, PIN_MOTOR_C_IN1,
                  PIN_MOTOR_C_IN2, motor_c_speed);
}

static int ultrasonic_read_mm(void)
{
    gpio_set_level(PIN_US_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(PIN_US_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_US_TRIG, 0);

    int64_t timeout_start = esp_timer_get_time();
    while (gpio_get_level(PIN_US_ECHO) == 0) {
        if (esp_timer_get_time() - timeout_start > ULTRASONIC_TIMEOUT_US) {
            return -1;
        }
    }

    const int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(PIN_US_ECHO) == 1) {
        if (esp_timer_get_time() - echo_start > ULTRASONIC_TIMEOUT_US) {
            return -1;
        }
    }

    const int64_t pulse_us = esp_timer_get_time() - echo_start;
    return (int)(pulse_us * 343 / 2000);
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

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  w/s  forward/backward\n");
    printf("  a/d  rotate left/right\n");
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

static void handle_command(char command)
{
    switch (tolower((unsigned char)command)) {
    case 'w':
        set_motors(-s_speed, 0, -s_speed);
        printf("Forward: A=-%d B=0 C=-%d\n", s_speed, s_speed);
        break;
    case 's':
        set_motors(s_speed, 0, s_speed);
        printf("Backward: A=%d B=0 C=%d\n", s_speed, s_speed);
        break;
    case 'a':
        set_motors(-s_speed, s_speed, 0);
        printf("Rotate left\n");
        break;
    case 'd':
        set_motors(s_speed, -s_speed, 0);
        printf("Rotate right\n");
        break;
    case '1':
        set_motors(s_speed, 0, 0);
        printf("Motor A only (right wheel)\n");
        break;
    case '2':
        set_motors(0, s_speed, 0);
        printf("Motor B only (rear wheel)\n");
        break;
    case '3':
        set_motors(0, 0, s_speed);
        printf("Motor C only (left wheel)\n");
        break;
    case 'r':
        run_short_test();
        break;
    case 'x':
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

    const int distance_mm = ultrasonic_read_mm();
    printf("IR=%d%d%d%d  distance=",
           gpio_get_level(PIN_IR_1),
           gpio_get_level(PIN_IR_2),
           gpio_get_level(PIN_IR_3),
           gpio_get_level(PIN_IR_4));
    if (distance_mm >= 0) {
        printf("%d mm", distance_mm);
    } else {
        printf("timeout");
    }
    printf("  encoder A=%" PRId64 " B=%" PRId64 " C=%" PRId64 "\n",
           motor_a_count, motor_b_count, motor_c_count);
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
    while (true) {
        char command;
        if (read(STDIN_FILENO, &command, 1) == 1) {
            handle_command(command);
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_telemetry_us) {
            print_telemetry();
            next_telemetry_us = now_us + TELEMETRY_PERIOD_MS * 1000LL;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
