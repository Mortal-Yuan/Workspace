#include "ultrasonic.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static int median(const int *values, unsigned count)
{
    int sorted[ULTRASONIC_MEDIAN_WINDOW] = {0};
    for (unsigned i = 0; i < count; ++i) {
        sorted[i] = values[i];
    }
    for (unsigned i = 1; i < count; ++i) {
        const int value = sorted[i];
        unsigned position = i;
        while (position > 0 && sorted[position - 1] > value) {
            sorted[position] = sorted[position - 1];
            --position;
        }
        sorted[position] = value;
    }
    return sorted[count / 2];
}

static void IRAM_ATTR echo_isr(void *arg)
{
    ultrasonic_t *sensor = arg;
    const int level = gpio_get_level(sensor->config.echo_pin);
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&sensor->lock);
    if (!sensor->active || sensor->capture_complete) {
        sensor->anomaly_count++;
        if (sensor->capture_complete) {
            sensor->anomaly = true;
        }
    } else if (level != 0) {
        if (sensor->rise_seen) {
            sensor->anomaly = true;
            sensor->anomaly_count++;
        } else {
            sensor->rise_seen = true;
            sensor->rise_us = now_us;
            sensor->captured_session = sensor->session_generation;
            sensor->captured_token = sensor->trigger_token;
        }
    } else if (!sensor->rise_seen ||
               sensor->captured_session != sensor->session_generation ||
               sensor->captured_token != sensor->trigger_token) {
        sensor->anomaly = true;
        sensor->anomaly_count++;
    } else {
        sensor->pulse_us = now_us - sensor->rise_us;
        sensor->capture_complete = true;
    }
    portEXIT_CRITICAL_ISR(&sensor->lock);
}

static void publish_event(ultrasonic_t *sensor, int64_t now_us,
                          bool has_echo, int64_t pulse_us,
                          bool echo_high, bool anomaly)
{
    ultrasonic_event_t event = {
        .seq = ++sensor->next_seq,
        .completed_us = now_us,
        .has_echo = has_echo,
        .pulse_us = (int32_t)pulse_us,
        .raw_mm = -1,
        .filtered_mm = sensor->snapshot.filtered_mm,
        .quality = ULTRASONIC_QUALITY_INVALID,
        .echo_high = echo_high,
        .safety_uncertain = anomaly,
    };

    if (has_echo) {
        event.raw_mm = (int32_t)(pulse_us * 343 / 2000);
        if (anomaly) {
            event.quality = ULTRASONIC_QUALITY_INVALID;
        } else if (event.raw_mm < 20 || event.raw_mm > 4000) {
            event.quality = ULTRASONIC_QUALITY_INVALID;
        } else {
            const bool jump = sensor->snapshot.filtered_mm >= 0 &&
                abs(event.raw_mm - sensor->snapshot.filtered_mm) >
                    sensor->config.timing.jump_mm;
            if (jump) {
                if (sensor->outlier_candidate_mm >= 0 &&
                    abs(event.raw_mm - sensor->outlier_candidate_mm) <=
                        sensor->config.timing.cluster_mm) {
                    sensor->outlier_count++;
                } else {
                    sensor->outlier_candidate_mm = event.raw_mm;
                    sensor->outlier_count = 1;
                }
                if (sensor->outlier_count <
                    (unsigned)sensor->config.timing.outlier_confirm_count) {
                    event.quality = ULTRASONIC_QUALITY_OUTLIER;
                    goto complete;
                }
                sensor->window_count = 0;
                sensor->window_next = 0;
            }
            sensor->outlier_candidate_mm = -1;
            sensor->outlier_count = 0;
            sensor->window[sensor->window_next] = event.raw_mm;
            sensor->window_next =
                (sensor->window_next + 1) % ULTRASONIC_MEDIAN_WINDOW;
            if (sensor->window_count < ULTRASONIC_MEDIAN_WINDOW) {
                sensor->window_count++;
            }
            event.filtered_mm = median(sensor->window, sensor->window_count);
            event.quality = ULTRASONIC_QUALITY_VALID;
            sensor->timeout_count = 0;
        }
    } else {
        sensor->timeout_count++;
        event.quality = sensor->timeout_count >=
            (unsigned)sensor->config.timing.lost_confirm_count ?
            ULTRASONIC_QUALITY_LOST : ULTRASONIC_QUALITY_OUTLIER;
        if (event.quality == ULTRASONIC_QUALITY_LOST) {
            event.filtered_mm = -1;
        }
        if (echo_high) {
            event.safety_uncertain = true;
        }
    }

complete:
    sensor->pending_event = event;
    sensor->event_pending = true;
    sensor->snapshot = (ultrasonic_snapshot_t) {
        .seq = event.seq,
        .updated_us = now_us,
        .raw_mm = event.raw_mm,
        .filtered_mm = event.filtered_mm,
        .quality = event.quality,
        .echo_high = echo_high,
        .waiting = false,
        .timeout_count = sensor->timeout_count,
        .anomaly_count = sensor->anomaly_count,
    };
}

static void start_trigger(ultrasonic_t *sensor, int64_t now_us)
{
    portENTER_CRITICAL(&sensor->lock);
    sensor->trigger_token++;
    sensor->active = true;
    sensor->rise_seen = false;
    sensor->capture_complete = false;
    sensor->anomaly = false;
    sensor->deadline_us = now_us + sensor->config.timing.timeout_us;
    portEXIT_CRITICAL(&sensor->lock);

    gpio_set_level(sensor->config.trigger_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(sensor->config.trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(sensor->config.trigger_pin, 0);
    sensor->next_trigger_us = now_us +
        sensor->config.timing.period_ms * 1000LL;
    sensor->snapshot.waiting = true;
}

esp_err_t ultrasonic_init(ultrasonic_t *sensor,
                          const ultrasonic_driver_config_t *config,
                          int64_t now_us)
{
    if (sensor == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;
    sensor->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    sensor->outlier_candidate_mm = -1;
    sensor->snapshot.raw_mm = -1;
    sensor->snapshot.filtered_mm = -1;
    sensor->snapshot.quality = ULTRASONIC_QUALITY_NEVER;

    const gpio_config_t trigger = {
        .pin_bit_mask = 1ULL << config->trigger_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&trigger);
    if (result != ESP_OK ||
        gpio_set_level(config->trigger_pin, 0) != ESP_OK) {
        return result != ESP_OK ? result : ESP_FAIL;
    }
    const gpio_config_t echo = {
        .pin_bit_mask = 1ULL << config->echo_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    result = gpio_config(&echo);
    if (result == ESP_OK) {
        result = gpio_isr_handler_add(config->echo_pin, echo_isr, sensor);
    }
    if (result != ESP_OK) {
        return result;
    }
    sensor->initialized = true;
    ultrasonic_restart_session(sensor, now_us);
    return ESP_OK;
}

void ultrasonic_restart_session(ultrasonic_t *sensor, int64_t now_us)
{
    if (sensor == NULL || !sensor->initialized) {
        return;
    }
    const bool echo_high = gpio_get_level(sensor->config.echo_pin) != 0;
    portENTER_CRITICAL(&sensor->lock);
    sensor->session_generation++;
    sensor->active = false;
    sensor->rise_seen = false;
    sensor->capture_complete = false;
    sensor->anomaly = false;
    portEXIT_CRITICAL(&sensor->lock);
    sensor->event_pending = false;
    sensor->blocked_echo_high = echo_high;
    sensor->next_trigger_us = now_us;
    if (echo_high) {
        publish_event(sensor, now_us, false, 0, true, true);
    }
}

void ultrasonic_step(ultrasonic_t *sensor, int64_t now_us)
{
    if (sensor == NULL || !sensor->initialized) {
        return;
    }
    bool completed = false;
    bool timed_out = false;
    bool anomaly = false;
    int64_t pulse_us = 0;
    const bool echo_high = gpio_get_level(sensor->config.echo_pin) != 0;

    portENTER_CRITICAL(&sensor->lock);
    if (sensor->capture_complete) {
        completed = true;
        pulse_us = sensor->pulse_us;
        anomaly = sensor->anomaly;
        sensor->capture_complete = false;
        sensor->active = false;
        sensor->rise_seen = false;
    } else if (sensor->active && now_us >= sensor->deadline_us) {
        timed_out = true;
        anomaly = sensor->anomaly;
        sensor->active = false;
        sensor->rise_seen = false;
        sensor->trigger_token++;
    }
    portEXIT_CRITICAL(&sensor->lock);

    if (!sensor->event_pending && completed) {
        publish_event(sensor, now_us, true, pulse_us, echo_high, anomaly);
    } else if (!sensor->event_pending && timed_out) {
        publish_event(sensor, now_us, false, 0, echo_high, anomaly);
    }

    if (sensor->blocked_echo_high) {
        if (!echo_high) {
            sensor->blocked_echo_high = false;
            sensor->next_trigger_us = now_us;
        }
        return;
    }
    if (!sensor->event_pending && !sensor->active &&
        now_us >= sensor->next_trigger_us) {
        start_trigger(sensor, now_us);
    }
}

bool ultrasonic_take_event(ultrasonic_t *sensor,
                           ultrasonic_event_t *event)
{
    if (sensor == NULL || event == NULL || !sensor->event_pending) {
        return false;
    }
    *event = sensor->pending_event;
    sensor->event_pending = false;
    return true;
}

ultrasonic_snapshot_t ultrasonic_snapshot(const ultrasonic_t *sensor)
{
    return sensor != NULL ? sensor->snapshot : (ultrasonic_snapshot_t) {
        .raw_mm = -1,
        .filtered_mm = -1,
        .quality = ULTRASONIC_QUALITY_NEVER,
    };
}
