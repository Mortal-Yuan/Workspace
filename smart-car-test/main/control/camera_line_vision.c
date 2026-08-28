#include "camera_line_vision.h"

#include <limits.h>
#include <string.h>

#define CAMERA_LINE_MAX_ANALYSIS_WIDTH 160

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static int divide_round_nearest(int64_t numerator, int64_t denominator)
{
    if (numerator >= 0) {
        return (int)((numerator + denominator / 2) / denominator);
    }
    return (int)-((-numerator + denominator / 2) / denominator);
}

static uint8_t grayscale_at(const uint8_t *pixels, size_t width,
                            size_t height, size_t x, size_t y,
                            bool rotate_180)
{
    if (rotate_180) {
        x = width - 1 - x;
        y = height - 1 - y;
    }
    const uint8_t *rgb = pixels + (y * width + x) * 3;
    return (uint8_t)((77U * rgb[0] + 150U * rgb[1] + 29U * rgb[2]) >> 8);
}

static int histogram_percentile(const uint32_t histogram[256],
                                uint32_t total, int permille)
{
    const uint32_t target =
        (uint32_t)(((uint64_t)total * (uint32_t)permille + 999U) / 1000U);
    uint32_t accumulated = 0;
    for (int value = 0; value < 256; ++value) {
        accumulated += histogram[value];
        if (accumulated >= target) return value;
    }
    return 255;
}

static int otsu_threshold(const uint32_t histogram[256], uint32_t total)
{
    uint64_t sum = 0;
    for (int value = 0; value < 256; ++value) {
        sum += (uint64_t)value * histogram[value];
    }

    uint32_t dark_count = 0;
    uint64_t dark_sum = 0;
    double best_score = -1.0;
    int best_threshold = 0;
    for (int value = 0; value < 255; ++value) {
        dark_count += histogram[value];
        dark_sum += (uint64_t)value * histogram[value];
        if (dark_count == 0) continue;
        const uint32_t light_count = total - dark_count;
        if (light_count == 0) break;
        const double dark_mean = (double)dark_sum / dark_count;
        const double light_mean = (double)(sum - dark_sum) / light_count;
        const double difference = dark_mean - light_mean;
        const double score = (double)dark_count * light_count *
                             difference * difference;
        if (score > best_score) {
            best_score = score;
            best_threshold = value;
        }
    }
    return best_threshold;
}

line_sensor_sample_t camera_line_virtual_sensors(int center_permille,
                                                 bool finish_detected)
{
    if (finish_detected) {
        return (line_sensor_sample_t) {true, true, true, true};
    }
    /* Split the cropped track window into five equal steering regions. */
    if (center_permille < -600) {
        return (line_sensor_sample_t) {.left = true};
    }
    if (center_permille < -200) {
        return (line_sensor_sample_t) {.left_center = true};
    }
    if (center_permille <= 200) {
        return (line_sensor_sample_t) {
            .left_center = true,
            .right_center = true,
        };
    }
    if (center_permille <= 600) {
        return (line_sensor_sample_t) {.right_center = true};
    }
    return (line_sensor_sample_t) {.right = true};
}

camera_line_analysis_t camera_line_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config)
{
    camera_line_analysis_t result = {0};
    if (pixels == NULL || config == NULL || width < 16 || height < 16 ||
        width > CAMERA_LINE_MAX_ANALYSIS_WIDTH ||
        config->roi_left_permille < 0 ||
        config->roi_right_permille > 1000 ||
        config->roi_left_permille >= config->roi_right_permille ||
        config->roi_top_permille < 0 ||
        config->roi_bottom_permille > 1000 ||
        config->roi_top_permille >= config->roi_bottom_permille) {
        return result;
    }

    const size_t roi_left =
        width * (size_t)config->roi_left_permille / 1000U;
    size_t roi_right =
        width * (size_t)config->roi_right_permille / 1000U;
    if (roi_right <= roi_left) roi_right = roi_left + 1;
    if (roi_right > width) roi_right = width;
    const size_t roi_top = height * (size_t)config->roi_top_permille / 1000U;
    size_t roi_bottom = height * (size_t)config->roi_bottom_permille / 1000U;
    if (roi_bottom <= roi_top) roi_bottom = roi_top + 1;
    if (roi_bottom > height) roi_bottom = height;
    const size_t roi_width = roi_right - roi_left;
    const size_t roi_height = roi_bottom - roi_top;
    if (roi_width < 2 || roi_height == 0) return result;
    const uint32_t roi_pixels = (uint32_t)(roi_width * roi_height);

    uint32_t histogram[256] = {0};
    for (size_t y = roi_top; y < roi_bottom; ++y) {
        for (size_t x = roi_left; x < roi_right; ++x) {
            ++histogram[grayscale_at(pixels, width, height, x, y,
                                     rotate_180)];
        }
    }

    result.valid = true;
    const int dark = histogram_percentile(histogram, roi_pixels, 20);
    const int light = histogram_percentile(histogram, roi_pixels, 900);
    result.contrast = (uint8_t)clamp_int(light - dark, 0, 255);
    if (result.contrast < config->minimum_contrast) return result;

    int threshold = otsu_threshold(histogram, roi_pixels) +
                    result.contrast / 8;
    threshold = clamp_int(threshold, dark, light - 1);
    result.threshold = (uint8_t)threshold;

    uint16_t column_counts[CAMERA_LINE_MAX_ANALYSIS_WIDTH] = {0};
    uint32_t black_pixels = 0;
    for (size_t y = roi_top; y < roi_bottom; ++y) {
        for (size_t x = roi_left; x < roi_right; ++x) {
            if (grayscale_at(pixels, width, height, x, y, rotate_180) <=
                result.threshold) {
                ++column_counts[x];
                ++black_pixels;
            }
        }
    }
    result.black_permille = (uint16_t)((uint64_t)black_pixels * 1000U /
                                       roi_pixels);

    bool active[CAMERA_LINE_MAX_ANALYSIS_WIDTH] = {0};
    for (size_t x = roi_left; x < roi_right; ++x) {
        active[x] = (uint32_t)column_counts[x] * 1000U >=
                    (uint32_t)roi_height *
                        (uint32_t)config->minimum_column_fill_permille;
    }
    for (size_t x = roi_left + 1; x + 1 < roi_right; ++x) {
        if (!active[x] && active[x - 1] && active[x + 1]) active[x] = true;
    }

    size_t best_left = 0;
    size_t best_right = 0;
    uint32_t best_pixels = 0;
    for (size_t x = roi_left; x < roi_right;) {
        if (!active[x]) {
            ++x;
            continue;
        }
        const size_t left = x;
        uint32_t component_pixels = 0;
        while (x < roi_right && active[x]) {
            component_pixels += column_counts[x];
            ++x;
        }
        const size_t right = x - 1;
        if (component_pixels > best_pixels) {
            best_pixels = component_pixels;
            best_left = left;
            best_right = right;
        }
    }
    if (best_pixels < (uint32_t)config->minimum_component_pixels) {
        return result;
    }

    uint64_t weighted_x = 0;
    uint32_t weights = 0;
    for (size_t x = best_left; x <= best_right; ++x) {
        weighted_x += (uint64_t)x * column_counts[x];
        weights += column_counts[x];
    }
    if (weights == 0) return result;

    /* Keep the weighted centroid at sub-pixel precision. Converting to an
     * integer column first biases even-width lines left and amplifies the
     * 80-pixel decoder's coarse horizontal quantization. */
    const uint64_t weighted_local_x =
        weighted_x - (uint64_t)roi_left * weights;
    const int64_t centered_twice =
        (int64_t)(2ULL * weighted_local_x) -
        (int64_t)(roi_width - 1) * weights;
    const int64_t center_denominator =
        (int64_t)(roi_width - 1) * weights;
    const int measured_center_permille = divide_round_nearest(
        centered_twice * 1000LL, center_denominator);
    result.center_permille = (int16_t)clamp_int(
        measured_center_permille + config->center_offset_permille,
        -1000, 1000);
    result.width_permille = (uint16_t)(
        (best_right - best_left + 1) * 1000U / roi_width);
    result.finish_detected =
        result.width_permille >= config->finish_width_permille &&
        result.black_permille >= config->finish_black_permille;
    result.line_detected = true;
    result.virtual_sensors = camera_line_virtual_sensors(
        result.center_permille, result.finish_detected);
    return result;
}
