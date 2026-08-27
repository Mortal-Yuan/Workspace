#include "status_display.h"

#include <stdio.h>
#include <string.h>

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

enum {
    DISPLAY_SPI_CLOCK_HZ = 8000000,
    DISPLAY_X_OFFSET = 0,
    DISPLAY_Y_OFFSET = 0,
};

#define RGB565(r, g, b) \
    ((uint16_t)((((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | ((b) >> 3)))

typedef struct {
    char character;
    uint8_t columns[5];
} glyph_t;

static const glyph_t GLYPHS[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'0', {0x3e, 0x51, 0x49, 0x45, 0x3e}},
    {'1', {0x00, 0x42, 0x7f, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4b, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7f, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3c, 0x4a, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1e}},
    {'A', {0x7e, 0x11, 0x11, 0x11, 0x7e}},
    {'B', {0x7f, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3e, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7f, 0x41, 0x41, 0x22, 0x1c}},
    {'E', {0x7f, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7f, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3e, 0x41, 0x49, 0x49, 0x7a}},
    {'H', {0x7f, 0x08, 0x08, 0x08, 0x7f}},
    {'I', {0x00, 0x41, 0x7f, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3f, 0x01}},
    {'K', {0x7f, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7f, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7f, 0x02, 0x0c, 0x02, 0x7f}},
    {'N', {0x7f, 0x04, 0x08, 0x10, 0x7f}},
    {'O', {0x3e, 0x41, 0x41, 0x41, 0x3e}},
    {'P', {0x7f, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3e, 0x41, 0x51, 0x21, 0x5e}},
    {'R', {0x7f, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7f, 0x01, 0x01}},
    {'U', {0x3f, 0x40, 0x40, 0x40, 0x3f}},
    {'V', {0x1f, 0x20, 0x40, 0x20, 0x1f}},
    {'W', {0x3f, 0x40, 0x38, 0x40, 0x3f}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

static uint16_t wire_color(uint16_t color)
{
    return (uint16_t)((color << 8) | (color >> 8));
}

static const uint8_t *glyph_columns(char character)
{
    for (size_t index = 0; index < sizeof(GLYPHS) / sizeof(GLYPHS[0]);
         ++index) {
        if (GLYPHS[index].character == character) {
            return GLYPHS[index].columns;
        }
    }
    return GLYPHS[0].columns;
}

static esp_err_t spi_write(spi_device_handle_t spi, bool data_phase,
                           const void *data, size_t length)
{
    if (length == 0) return ESP_OK;
    gpio_set_level(BOARD_PIN_DISPLAY_DC, data_phase ? 1 : 0);
    spi_transaction_t transaction = {
        .length = length * 8,
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(spi, &transaction);
}

static esp_err_t write_command(spi_device_handle_t spi, uint8_t command,
                               const uint8_t *parameters,
                               size_t parameter_count)
{
    esp_err_t result = spi_write(spi, false, &command, 1);
    if (result == ESP_OK && parameter_count > 0) {
        result = spi_write(spi, true, parameters, parameter_count);
    }
    return result;
}

static esp_err_t panel_initialize(spi_device_handle_t *spi)
{
    gpio_config_t control_config = {
        .pin_bit_mask = (1ULL << BOARD_PIN_DISPLAY_DC) |
                        (1ULL << BOARD_PIN_DISPLAY_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&control_config);
    if (result != ESP_OK) return result;
    gpio_set_level(BOARD_PIN_DISPLAY_DC, 0);
    gpio_set_level(BOARD_PIN_DISPLAY_RESET, 0);

    spi_bus_config_t bus_config = {
        .mosi_io_num = BOARD_PIN_DISPLAY_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_PIN_DISPLAY_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = STATUS_DISPLAY_WIDTH *
                           STATUS_DISPLAY_LINES_PER_TRANSFER * 2,
    };
    result = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) return result;

    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = DISPLAY_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = BOARD_PIN_DISPLAY_CS,
        .queue_size = 1,
    };
    result = spi_bus_add_device(SPI2_HOST, &device_config, spi);
    if (result != ESP_OK) return result;

    /* LQ_TFT18SPIV33 requires the ILI9163B hardware-reset sequence. */
    gpio_set_level(BOARD_PIN_DISPLAY_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BOARD_PIN_DISPLAY_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    result = write_command(*spi, 0x11, NULL, 0);  /* Sleep out. */
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(20));

    const uint8_t color_mode = 0x55;  /* RGB565. */
    result = write_command(*spi, 0x3a, &color_mode, 1);
    if (result != ESP_OK) return result;

    const uint8_t gamma_curve = 0x04;
    result = write_command(*spi, 0x26, &gamma_curve, 1);
    if (result != ESP_OK) return result;
    const uint8_t gamma_adjust = 0x01;
    result = write_command(*spi, 0xf2, &gamma_adjust, 1);
    if (result != ESP_OK) return result;

    const uint8_t positive_gamma[] = {
        0x3f, 0x25, 0x1c, 0x1e, 0x20, 0x12, 0x2a, 0x90,
        0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    result = write_command(*spi, 0xe0, positive_gamma,
                           sizeof(positive_gamma));
    if (result != ESP_OK) return result;
    const uint8_t negative_gamma[] = {
        0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xa7,
        0x3d, 0x18, 0x25, 0x2a, 0x2b, 0x2b, 0x3a,
    };
    result = write_command(*spi, 0xe1, negative_gamma,
                           sizeof(negative_gamma));
    if (result != ESP_OK) return result;

    const uint8_t frame_rate[] = {0x00, 0x00};
    result = write_command(*spi, 0xb1, frame_rate, sizeof(frame_rate));
    if (result != ESP_OK) return result;
    const uint8_t inversion_control = 0x07;
    result = write_command(*spi, 0xb4, &inversion_control, 1);
    if (result != ESP_OK) return result;
    const uint8_t power_control_1[] = {0x0a, 0x02};
    result = write_command(*spi, 0xc0, power_control_1,
                           sizeof(power_control_1));
    if (result != ESP_OK) return result;
    const uint8_t power_control_2 = 0x02;
    result = write_command(*spi, 0xc1, &power_control_2, 1);
    if (result != ESP_OK) return result;
    const uint8_t vcom_control[] = {0x4f, 0x5a};
    result = write_command(*spi, 0xc5, vcom_control,
                           sizeof(vcom_control));
    if (result != ESP_OK) return result;
    const uint8_t vcom_offset = 0x40;
    result = write_command(*spi, 0xc7, &vcom_offset, 1);
    if (result != ESP_OK) return result;

    const uint8_t controller_columns[] = {0x00, 0x00, 0x00, 0xa8};
    result = write_command(*spi, 0x2a, controller_columns,
                           sizeof(controller_columns));
    if (result != ESP_OK) return result;
    const uint8_t controller_rows[] = {0x00, 0x00, 0x00, 0xb3};
    result = write_command(*spi, 0x2b, controller_rows,
                           sizeof(controller_rows));
    if (result != ESP_OK) return result;

    const uint8_t memory_access = 0xa0;  /* Landscape, RGB order. */
    result = write_command(*spi, 0x36, &memory_access, 1);
    if (result != ESP_OK) return result;
    const uint8_t source_output = 0x00;
    result = write_command(*spi, 0xb7, &source_output, 1);
    if (result != ESP_OK) return result;
    result = write_command(*spi, 0x29, NULL, 0);  /* Display on. */
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(20));
    result = write_command(*spi, 0x2c, NULL, 0);  /* Memory write. */
    return ESP_OK;
}

static esp_err_t set_window(spi_device_handle_t spi, int x_start,
                            int y_start, int x_end, int y_end)
{
    x_start += DISPLAY_X_OFFSET;
    x_end += DISPLAY_X_OFFSET;
    y_start += DISPLAY_Y_OFFSET;
    y_end += DISPLAY_Y_OFFSET;
    const uint8_t columns[] = {
        (uint8_t)(x_start >> 8), (uint8_t)x_start,
        (uint8_t)(x_end >> 8), (uint8_t)x_end,
    };
    const uint8_t rows[] = {
        (uint8_t)(y_start >> 8), (uint8_t)y_start,
        (uint8_t)(y_end >> 8), (uint8_t)y_end,
    };
    esp_err_t result = write_command(spi, 0x2a, columns, sizeof(columns));
    if (result == ESP_OK) {
        result = write_command(spi, 0x2b, rows, sizeof(rows));
    }
    if (result == ESP_OK) {
        result = write_command(spi, 0x2c, NULL, 0);
    }
    return result;
}

static void put_pixel(status_display_t *display, int chunk_y,
                      int x, int y, uint16_t color)
{
    if (x < 0 || x >= STATUS_DISPLAY_WIDTH ||
        y < chunk_y || y >= chunk_y + STATUS_DISPLAY_LINES_PER_TRANSFER ||
        y >= STATUS_DISPLAY_HEIGHT) {
        return;
    }
    display->pixel_buffer[(y - chunk_y) * STATUS_DISPLAY_WIDTH + x] =
        wire_color(color);
}

static void fill_rectangle(status_display_t *display, int chunk_y,
                           int x, int y, int width, int height,
                           uint16_t color)
{
    for (int py = y; py < y + height; ++py) {
        for (int px = x; px < x + width; ++px) {
            put_pixel(display, chunk_y, px, py, color);
        }
    }
}

static void draw_text(status_display_t *display, int chunk_y,
                      int x, int y, const char *text, int scale,
                      uint16_t color)
{
    while (*text != '\0') {
        const uint8_t *columns = glyph_columns(*text++);
        for (int column = 0; column < 5; ++column) {
            for (int row = 0; row < 7; ++row) {
                if ((columns[column] & (1U << row)) == 0) continue;
                fill_rectangle(display, chunk_y,
                               x + column * scale, y + row * scale,
                               scale, scale, color);
            }
        }
        x += 6 * scale;
    }
}

static const char *mode_text(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_IDLE: return "IDLE";
    case APP_MODE_AUTONOMOUS: return "AUTO";
    case APP_MODE_MANUAL: return "MANUAL";
    case APP_MODE_SELF_TEST: return "TEST";
    case APP_MODE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

static uint16_t status_color(const status_display_snapshot_t *snapshot)
{
    if (snapshot->failsafe || snapshot->mode == APP_MODE_FAULT) {
        return RGB565(220, 30, 30);
    }
    if (snapshot->finished) return RGB565(30, 160, 220);
    if (snapshot->obstacle_state > 1) return RGB565(235, 150, 20);
    return snapshot->mode == APP_MODE_AUTONOMOUS ?
           RGB565(20, 180, 75) : RGB565(80, 85, 95);
}

static esp_err_t render_screen(status_display_t *display,
                               spi_device_handle_t spi,
                               const status_display_snapshot_t *snapshot)
{
    char line[32];
    for (int chunk_y = 0; chunk_y < STATUS_DISPLAY_HEIGHT;
         chunk_y += STATUS_DISPLAY_LINES_PER_TRANSFER) {
        for (int local_y = 0;
             local_y < STATUS_DISPLAY_LINES_PER_TRANSFER; ++local_y) {
            const int y = chunk_y + local_y;
            uint16_t color = RGB565(8, 15, 25);
            if (y < 20) color = RGB565(15, 70, 135);
            if (y >= 112) color = status_color(snapshot);
            for (int x = 0; x < STATUS_DISPLAY_WIDTH; ++x) {
                display->pixel_buffer[local_y * STATUS_DISPLAY_WIDTH + x] =
                    wire_color(color);
            }
        }

        draw_text(display, chunk_y, 27, 3, "SMART CAR", 2,
                  RGB565(255, 255, 255));
        snprintf(line, sizeof(line), "MODE %s", mode_text(snapshot->mode));
        draw_text(display, chunk_y, 5, 25, line, 1,
                  RGB565(210, 225, 240));

        draw_text(display, chunk_y, 5, 42, "IR", 1,
                  RGB565(210, 225, 240));
        for (int bit = 0; bit < 4; ++bit) {
            const bool active = (snapshot->line_pattern & (1U << (3 - bit))) != 0;
            fill_rectangle(display, chunk_y, 34 + bit * 31, 38,
                           24, 16, active ? RGB565(245, 245, 245) :
                                            RGB565(45, 55, 65));
        }

        if (snapshot->ultrasonic_mm >= 0) {
            snprintf(line, sizeof(line), "US %ldMM",
                     (long)snapshot->ultrasonic_mm);
        } else {
            snprintf(line, sizeof(line), "US ----MM");
        }
        draw_text(display, chunk_y, 5, 62, line, 2,
                  snapshot->ultrasonic_mm >= 20 &&
                  snapshot->ultrasonic_mm <= 100 ? RGB565(255, 70, 70) :
                                                   RGB565(90, 220, 235));

        snprintf(line, sizeof(line), "OBS %u", snapshot->obstacle_state);
        draw_text(display, chunk_y, 5, 84, line, 1,
                  RGB565(245, 190, 70));
        snprintf(line, sizeof(line), "M %d %d %d",
                 snapshot->motor.a, snapshot->motor.b, snapshot->motor.c);
        draw_text(display, chunk_y, 5, 99, line, 1,
                  RGB565(180, 195, 210));

        const char *footer = snapshot->finished ? "FINISHED" :
                             snapshot->failsafe ? "FAILSAFE" :
                             snapshot->mode == APP_MODE_AUTONOMOUS ?
                             "RUNNING" : "READY";
        const int footer_width = (int)strlen(footer) * 12;
        draw_text(display, chunk_y,
                  (STATUS_DISPLAY_WIDTH - footer_width) / 2,
                  115, footer, 2,
                  RGB565(255, 255, 255));

        const int height = chunk_y + STATUS_DISPLAY_LINES_PER_TRANSFER >
                           STATUS_DISPLAY_HEIGHT ?
                           STATUS_DISPLAY_HEIGHT - chunk_y :
                           STATUS_DISPLAY_LINES_PER_TRANSFER;
        esp_err_t result = set_window(spi, 0, chunk_y,
                                      STATUS_DISPLAY_WIDTH - 1,
                                      chunk_y + height - 1);
        if (result != ESP_OK) return result;
        result = spi_write(spi, true, display->pixel_buffer,
                           STATUS_DISPLAY_WIDTH * height * sizeof(uint16_t));
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}

static void display_task(void *argument)
{
    status_display_t *display = argument;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    spi_device_handle_t spi = NULL;
    display->last_error = panel_initialize(&spi);
    if (display->last_error != ESP_OK) {
        display->active = false;
        vTaskDelete(NULL);
        return;
    }
    display->active = true;

    status_display_snapshot_t snapshot = {
        .mode = APP_MODE_IDLE,
        .ultrasonic_mm = -1,
    };
    while (true) {
        status_display_snapshot_t latest;
        if (xQueueReceive(display->queue, &latest,
                          pdMS_TO_TICKS(500)) == pdTRUE) {
            snapshot = latest;
            while (xQueueReceive(display->queue, &latest, 0) == pdTRUE) {
                snapshot = latest;
            }
        }
        display->last_error = render_screen(display, spi, &snapshot);
        if (display->last_error != ESP_OK) {
            display->active = false;
            vTaskDelete(NULL);
            return;
        }
    }
}

esp_err_t status_display_init(status_display_t *display)
{
    if (display == NULL) return ESP_ERR_INVALID_ARG;
    memset(display, 0, sizeof(*display));
    /* Keep the panel blank in IDLE and after a controller reset. */
    const gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << BOARD_PIN_DISPLAY_RESET,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&reset_config);
    if (result != ESP_OK) return result;
    gpio_set_level(BOARD_PIN_DISPLAY_RESET, 0);
    display->last_error = ESP_ERR_INVALID_STATE;
    display->queue = xQueueCreateStatic(
        1, sizeof(status_display_snapshot_t), display->queue_storage,
        &display->queue_static);
    if (display->queue == NULL) return ESP_ERR_NO_MEM;
    display->task = xTaskCreateStatic(
        display_task, "status_display", STATUS_DISPLAY_TASK_STACK,
        display, 1, display->task_stack, &display->task_buffer);
    return display->task != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

bool status_display_enable(status_display_t *display)
{
    if (display == NULL || display->task == NULL) return false;
    if (display->requested) return true;
    display->requested = true;
    xTaskNotifyGive(display->task);
    return true;
}

void status_display_publish(status_display_t *display,
                            const status_display_snapshot_t *snapshot)
{
    if (display == NULL || snapshot == NULL || !display->requested ||
        display->queue == NULL) {
        return;
    }
    xQueueOverwrite(display->queue, snapshot);
}

bool status_display_is_requested(const status_display_t *display)
{
    return display != NULL && display->requested;
}

bool status_display_is_active(const status_display_t *display)
{
    return display != NULL && display->active;
}

esp_err_t status_display_last_error(const status_display_t *display)
{
    return display != NULL ? display->last_error : ESP_ERR_INVALID_ARG;
}
