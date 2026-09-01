#include "camera_ball_vision.h"

#include <limits.h>

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static const uint8_t *pixel_at(const uint8_t *pixels, size_t width,
                               size_t height, size_t x, size_t y,
                               bool rotate_180)
{
    if (rotate_180) {
        x = width - 1U - x;
        y = height - 1U - y;
    }
    return pixels + (y * width + x) * 3U;
}

static bool is_red(const uint8_t *rgb,
                   const camera_ball_config_t *config)
{
    const int red = rgb[0];
    const int green = rgb[1];
    const int blue = rgb[2];
    const int total = red + green + blue;
    const int competitor = green > blue ? green : blue;
    return red >= config->red_minimum &&
        red - competitor >= config->red_dominance && total > 0 &&
        red * 1000 >= config->red_ratio_permille * total;
}

camera_ball_observation_t camera_ball_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace)
{
    camera_ball_observation_t result = {0};
    if (pixels == NULL || config == NULL || workspace == NULL ||
        width == 0 || height == 0 ||
        width > CAMERA_BALL_VISION_MAX_WIDTH ||
        height > CAMERA_BALL_VISION_MAX_HEIGHT) {
        return result;
    }
    result.valid = true;

    const size_t pixel_count = width * height;
    uint8_t *mask = workspace->mask;
    int best_red_score = INT_MIN;
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            const uint8_t *rgb = pixel_at(
                pixels, width, height, x, y, rotate_180);
            const int red_score = 2 * (int)rgb[0] - rgb[1] - rgb[2];
            if (red_score > best_red_score) {
                best_red_score = red_score;
                result.probe_red_score = (int16_t)red_score;
                result.probe_x_permille = (int16_t)clamp_int(
                    (int)(x * 2000U / width) - 1000, -1000, 1000);
                result.probe_y_permille = (uint16_t)clamp_int(
                    (int)(y * 1000U / height), 0, 1000);
                result.probe_red = rgb[0];
                result.probe_green = rgb[1];
                result.probe_blue = rgb[2];
            }
            mask[y * width + x] = is_red(rgb, config) ? 1U : 0U;
        }
    }

    uint32_t best_area = 0;
    uint32_t best_sum_x = 0;
    uint32_t best_sum_y = 0;
    uint32_t best_sum_r = 0;
    uint32_t best_sum_g = 0;
    uint32_t best_sum_b = 0;
    size_t best_minimum_x = 0;
    size_t best_maximum_x = 0;
    size_t best_minimum_y = 0;
    size_t best_maximum_y = 0;
    uint16_t best_fill = 0;
    uint16_t best_roundness = 0;
    bool best_is_ball = false;

    for (size_t start = 0; start < pixel_count; ++start) {
        if (mask[start] != 1U) continue;
        if (result.component_count < UINT8_MAX) {
            ++result.component_count;
        }

        uint32_t head = 0;
        uint32_t tail = 0;
        workspace->queue[tail++] = (uint16_t)start;
        mask[start] = 2U;
        uint32_t area = 0;
        uint32_t sum_x = 0;
        uint32_t sum_y = 0;
        uint32_t sum_r = 0;
        uint32_t sum_g = 0;
        uint32_t sum_b = 0;
        size_t minimum_x = width;
        size_t maximum_x = 0;
        size_t minimum_y = height;
        size_t maximum_y = 0;

        while (head < tail) {
            const size_t index = workspace->queue[head++];
            const size_t y = index / width;
            const size_t x = index - y * width;
            const uint8_t *rgb = pixel_at(
                pixels, width, height, x, y, rotate_180);
            ++area;
            sum_x += (uint32_t)x;
            sum_y += (uint32_t)y;
            sum_r += rgb[0];
            sum_g += rgb[1];
            sum_b += rgb[2];
            if (x < minimum_x) minimum_x = x;
            if (x > maximum_x) maximum_x = x;
            if (y < minimum_y) minimum_y = y;
            if (y > maximum_y) maximum_y = y;

            for (int dy = -1; dy <= 1; ++dy) {
                const int next_y = (int)y + dy;
                if (next_y < 0 || next_y >= (int)height) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int next_x = (int)x + dx;
                    if (next_x < 0 || next_x >= (int)width) continue;
                    const size_t next =
                        (size_t)next_y * width + (size_t)next_x;
                    if (mask[next] == 1U) {
                        mask[next] = 2U;
                        workspace->queue[tail++] = (uint16_t)next;
                    }
                }
            }
        }

        const size_t box_width = maximum_x - minimum_x + 1U;
        const size_t box_height = maximum_y - minimum_y + 1U;
        const uint16_t area_permille = (uint16_t)(
            (uint64_t)area * 1000U / pixel_count);
        const uint16_t fill_permille = (uint16_t)(
            (uint64_t)area * 1000U / (box_width * box_height));
        const uint16_t roundness_permille = (uint16_t)(
            (box_width < box_height ? box_width : box_height) * 1000U /
            (box_width > box_height ? box_width : box_height));
        const int mean_red = (int)(sum_r / area);
        const int mean_green = (int)(sum_g / area);
        const int mean_blue = (int)(sum_b / area);
        const int mean_competitor =
            mean_green > mean_blue ? mean_green : mean_blue;
        const bool ball_shape =
            area_permille >= config->minimum_area_permille &&
            fill_permille >= config->minimum_fill_permille &&
            roundness_permille >= config->minimum_roundness_permille &&
            mean_red - mean_competitor >=
                config->minimum_mean_red_dominance &&
            minimum_x >= (size_t)config->edge_margin_pixels &&
            minimum_y >= (size_t)config->edge_margin_pixels &&
            maximum_x + (size_t)config->edge_margin_pixels < width &&
            maximum_y + (size_t)config->edge_margin_pixels < height;
        if ((ball_shape && best_is_ball && area <= best_area) ||
            (!ball_shape && (best_is_ball || area <= best_area))) {
            continue;
        }

        best_area = area;
        best_is_ball = ball_shape;
        best_sum_x = sum_x;
        best_sum_y = sum_y;
        best_sum_r = sum_r;
        best_sum_g = sum_g;
        best_sum_b = sum_b;
        best_minimum_x = minimum_x;
        best_maximum_x = maximum_x;
        best_minimum_y = minimum_y;
        best_maximum_y = maximum_y;
        best_fill = fill_permille;
        best_roundness = roundness_permille;
    }

    if (best_area == 0) return result;

    const size_t box_width = best_maximum_x - best_minimum_x + 1U;
    const size_t box_height = best_maximum_y - best_minimum_y + 1U;
    const int center_x = (int)((uint64_t)best_sum_x * 2000U /
                               (best_area * width)) - 1000;
    result.candidate = best_is_ball;
    result.color = BALL_COLOR_RED;
    result.center_x_permille = (int16_t)clamp_int(center_x, -1000, 1000);
    result.center_y_permille = (uint16_t)clamp_int(
        (int)((uint64_t)best_sum_y * 1000U /
              (best_area * height)), 0, 1000);
    result.width_permille = (uint16_t)(box_width * 1000U / width);
    result.height_permille = (uint16_t)(box_height * 1000U / height);
    result.area_permille = (uint16_t)(
        (uint64_t)best_area * 1000U / pixel_count);
    result.fill_permille = best_fill;
    result.roundness_permille = best_roundness;
    result.mean_red = (uint8_t)(best_sum_r / best_area);
    result.mean_green = (uint8_t)(best_sum_g / best_area);
    result.mean_blue = (uint8_t)(best_sum_b / best_area);
    result.matched_pixels = best_area > UINT16_MAX ?
        UINT16_MAX : (uint16_t)best_area;

    const int color_margin = result.mean_red -
        (result.mean_green > result.mean_blue ?
         result.mean_green : result.mean_blue);
    const int color_score = clamp_int(
        color_margin * 1000 / (config->red_dominance * 4), 0, 1000);
    const int area_score = clamp_int(
        result.area_permille * 1000 /
        (config->minimum_area_permille * 8), 0, 1000);
    result.confidence_permille = (uint16_t)(
        (color_score * 4 + best_fill * 2 + best_roundness * 2 +
         area_score * 2) / 10);
    return result;
}
