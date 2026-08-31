#include "camera_line_vision.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define CAMERA_LINE_MAX_ANALYSIS_WIDTH 160

enum {
    /* This floor keeps component area meaningful even for a perfect history
     * match.  History changes ranking smoothly; it never rejects a jump. */
    CAMERA_LINE_HISTORY_ERROR_FLOOR = 250,
    CAMERA_LINE_HISTORY_SCORE_SCALE = 1000000,
};

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

int camera_line_steering_from_geometry(
    int near_center_permille, int far_center_permille,
    const camera_line_config_t *config)
{
    if (config == NULL) return 0;
    const int coordinate_limit = config->horizontal_scale_permille;
    const int heading = clamp_int(
        far_center_permille - near_center_permille,
        -coordinate_limit * 2, coordinate_limit * 2);
    int heading_gain = config->heading_gain_permille;
    /* On a hairpin the near and far thirds can lie on opposite sides of the
     * image.  The ordinary lookahead term then cancels the near-line error and
     * falsely requests almost-straight motion.  Keep the connected component,
     * but temporarily trust its near third more strongly. */
    const bool crosses_center =
        (near_center_permille < 0 && far_center_permille > 0) ||
        (near_center_permille > 0 && far_center_permille < 0);
    if (crosses_center &&
        abs(near_center_permille) >=
            config->hairpin_near_threshold_permille &&
        abs(heading) >= config->hairpin_heading_threshold_permille) {
        heading_gain = config->hairpin_heading_gain_permille;
    }
    return clamp_int(
        near_center_permille + heading * heading_gain / 1000,
        -coordinate_limit * 2, coordinate_limit * 2);
}

static int center_from_pixels(uint64_t sum_x, uint32_t count,
                              size_t roi_width)
{
    if (count == 0 || roi_width < 2) return 0;
    const int64_t centered_twice =
        (int64_t)(2ULL * sum_x) -
        (int64_t)(roi_width - 1) * count;
    const int64_t denominator = (int64_t)(roi_width - 1) * count;
    return divide_round_nearest(centered_twice * 1000LL, denominator);
}

static camera_line_analysis_t camera_line_analyze_rgb888_internal(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace,
    bool has_previous_line, int previous_center_permille,
    int previous_steering_permille)
{
    camera_line_analysis_t result = {0};
    if (pixels == NULL || config == NULL || workspace == NULL ||
        width < 16 || height < 16 ||
        width > CAMERA_LINE_MAX_ANALYSIS_WIDTH ||
        height > CAMERA_LINE_VISION_MAX_HEIGHT ||
        config->roi_left_permille < 0 ||
        config->roi_right_permille > 1000 ||
        config->roi_left_permille >= config->roi_right_permille ||
        config->roi_top_permille < 0 ||
        config->roi_bottom_permille > 1000 ||
        config->roi_top_permille >= config->roi_bottom_permille ||
        config->horizontal_scale_permille < 1000 ||
        config->horizontal_scale_permille > 4000) {
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
    if (roi_pixels > CAMERA_LINE_VISION_MAX_ROI_PIXELS) return result;

    uint32_t *histogram = workspace->histogram;
    memset(histogram, 0, sizeof(workspace->histogram));
    for (size_t y = roi_top; y < roi_bottom; ++y) {
        for (size_t x = roi_left; x < roi_right; ++x) {
            ++histogram[grayscale_at(pixels, width, height, x, y,
                                     rotate_180)];
        }
    }

    result.valid = true;
    /* Contrast and thresholding create the binary image; candidate acceptance
     * below depends only on eight-neighbour connected components. */
    const int dark = histogram_percentile(histogram, roi_pixels, 20);
    const int light = histogram_percentile(histogram, roi_pixels, 900);
    result.contrast = (uint8_t)clamp_int(light - dark, 0, 255);
    if (result.contrast < config->minimum_contrast) return result;

    int threshold = otsu_threshold(histogram, roi_pixels) +
                    result.contrast / 8;
    threshold = clamp_int(threshold, dark, light - 1);
    result.threshold = (uint8_t)threshold;

    uint8_t *mask = workspace->mask;
    uint16_t *queue = workspace->queue;
    memset(mask, 0, roi_pixels);
    uint32_t black_pixels = 0;
    for (size_t y = roi_top; y < roi_bottom; ++y) {
        for (size_t x = roi_left; x < roi_right; ++x) {
            if (grayscale_at(pixels, width, height, x, y, rotate_180) <=
                result.threshold) {
                mask[(y - roi_top) * roi_width + (x - roi_left)] = 1;
                ++black_pixels;
            }
        }
    }
    result.black_permille = (uint16_t)((uint64_t)black_pixels * 1000U /
                                       roi_pixels);

    uint32_t best_area_pixels = 0;
    uint64_t best_selection_score = 0;
    bool found = false;
    int best_near_center = 0;
    int best_far_center = 0;
    int best_heading = 0;
    int best_steering = 0;
    uint16_t best_width_permille = 0;
    uint16_t best_height_permille = 0;
    uint16_t best_area_permille = 0;
    bool best_finish = false;

    for (uint32_t start = 0; start < roi_pixels; ++start) {
        if (mask[start] != 1) continue;
        if (result.connected_component_count < UINT8_MAX) {
            ++result.connected_component_count;
        }

        uint32_t *row_counts = workspace->row_counts;
        uint64_t *row_sum_x = workspace->row_sum_x;
        memset(row_counts, 0, sizeof(workspace->row_counts));
        memset(row_sum_x, 0, sizeof(workspace->row_sum_x));
        uint32_t head = 0;
        uint32_t tail = 0;
        queue[tail++] = (uint16_t)start;
        mask[start] = 2;
        uint32_t area = 0;
        size_t minimum_y = roi_height;
        size_t maximum_y = 0;

        while (head < tail) {
            const uint32_t index = queue[head++];
            const size_t local_y = index / roi_width;
            const size_t local_x = index - local_y * roi_width;
            ++area;
            ++row_counts[local_y];
            row_sum_x[local_y] += local_x;
            if (local_y < minimum_y) minimum_y = local_y;
            if (local_y > maximum_y) maximum_y = local_y;

            for (int dy = -1; dy <= 1; ++dy) {
                const int next_y = (int)local_y + dy;
                if (next_y < 0 || next_y >= (int)roi_height) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int next_x = (int)local_x + dx;
                    if (next_x < 0 || next_x >= (int)roi_width) continue;
                    const uint32_t next =
                        (uint32_t)next_y * (uint32_t)roi_width +
                        (uint32_t)next_x;
                    if (mask[next] == 1) {
                        mask[next] = 2;
                        queue[tail++] = (uint16_t)next;
                    }
                }
            }
        }

        const size_t component_height = maximum_y - minimum_y + 1;
        const size_t component_near_rows = (component_height + 2U) / 3U;
        const size_t component_near_start =
            maximum_y + 1U - component_near_rows;
        uint32_t near_count = 0;
        uint64_t near_sum_x = 0;
        uint32_t maximum_row_pixels = 0;
        for (size_t row = minimum_y; row <= maximum_y; ++row) {
            if (row_counts[row] > maximum_row_pixels) {
                maximum_row_pixels = row_counts[row];
            }
            if (row >= component_near_start) {
                near_count += row_counts[row];
                near_sum_x += row_sum_x[row];
            }
        }
        const uint16_t height_permille = (uint16_t)(
            component_height * 1000U / roi_height);
        /* Preserve the configured coordinate scale for diagnostics, steering,
         * and the separate finish-line classification. */
        const uint16_t width_permille = (uint16_t)(
            (uint64_t)maximum_row_pixels *
            (uint32_t)config->horizontal_scale_permille / roi_width);
        const uint16_t area_permille = (uint16_t)(
            (uint64_t)area *
            (uint32_t)config->horizontal_scale_permille / roi_pixels);
        const bool finish_shape =
            width_permille >= config->finish_width_permille &&
            area_permille >= config->finish_black_permille;
        const size_t far_end = minimum_y +
            (component_height + 2U) / 3U;
        uint32_t far_count = 0;
        uint64_t far_sum_x = 0;
        for (size_t row = minimum_y;
             row < far_end && row <= maximum_y; ++row) {
            far_count += row_counts[row];
            far_sum_x += row_sum_x[row];
        }
        if (far_count == 0) continue;

        const int coordinate_limit = config->horizontal_scale_permille;
        const int near_center = clamp_int(
            center_from_pixels(near_sum_x, near_count, roi_width) *
                config->horizontal_scale_permille / 1000 +
                config->center_offset_permille,
            -coordinate_limit, coordinate_limit);
        const int far_center = clamp_int(
            center_from_pixels(far_sum_x, far_count, roi_width) *
                config->horizontal_scale_permille / 1000 +
                config->center_offset_permille,
            -coordinate_limit, coordinate_limit);
        const int heading = clamp_int(
            far_center - near_center,
            -coordinate_limit * 2, coordinate_limit * 2);
        const int steering = camera_line_steering_from_geometry(
            near_center, far_center, config);
        /* Every non-empty connected component remains eligible.  A reliable
         * previous line only soft-ranks multiple candidates: area is divided
         * by a smooth position/direction error term, with no jump cutoff. */
        uint64_t selection_score = area;
        if (has_previous_line) {
            const uint32_t position_error = (uint32_t)abs(
                near_center - previous_center_permille);
            const uint32_t direction_error = (uint32_t)abs(
                steering - previous_steering_permille);
            const uint32_t history_denominator =
                CAMERA_LINE_HISTORY_ERROR_FLOOR +
                position_error + direction_error;
            selection_score =
                (uint64_t)area * CAMERA_LINE_HISTORY_SCORE_SCALE /
                history_denominator;
        }
        if (!found || selection_score > best_selection_score ||
            (selection_score == best_selection_score &&
             area > best_area_pixels)) {
            found = true;
            best_area_pixels = area;
            best_selection_score = selection_score;
            best_near_center = near_center;
            best_far_center = far_center;
            best_heading = heading;
            best_steering = steering;
            best_width_permille = width_permille;
            best_height_permille = height_permille;
            best_area_permille = area_permille;
            best_finish = finish_shape;
        }
    }

    if (!found) return result;
    result.center_permille = (int16_t)best_near_center;
    result.far_center_permille = (int16_t)best_far_center;
    result.heading_permille = (int16_t)best_heading;
    result.steering_permille = (int16_t)best_steering;
    result.width_permille = best_width_permille;
    result.component_height_permille = best_height_permille;
    result.component_area_permille = best_area_permille;
    result.finish_detected = best_finish;
    result.line_detected = true;
    result.virtual_sensors = camera_line_virtual_sensors(
        result.steering_permille, result.finish_detected);
    return result;
}

camera_line_analysis_t camera_line_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace)
{
    return camera_line_analyze_rgb888_with_hint(
        pixels, width, height, rotate_180, config, workspace,
        false, 0, 0);
}

camera_line_analysis_t camera_line_analyze_rgb888_with_hint(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace,
    bool has_previous_line, int previous_center_permille,
    int previous_steering_permille)
{
    return camera_line_analyze_rgb888_internal(
        pixels, width, height, rotate_180, config, workspace,
        has_previous_line, previous_center_permille,
        previous_steering_permille);
}

camera_line_analysis_t camera_line_analyze_lower_half_rgb888(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace)
{
    return camera_line_analyze_lower_half_rgb888_with_hint(
        pixels, width, height, rotate_180, config, workspace,
        false, 0, 0);
}

camera_line_analysis_t camera_line_analyze_lower_half_rgb888_with_hint(
    const uint8_t *pixels, size_t width, size_t height,
    bool rotate_180, const camera_line_config_t *config,
    camera_line_vision_workspace_t *workspace,
    bool has_previous_line, int previous_center_permille,
    int previous_steering_permille)
{
    camera_line_analysis_t result = {0};
    if (pixels == NULL || config == NULL || height < 2) return result;

    const int lower_half_top_permille = 500;
    const int full_roi_top = config->roi_top_permille <
        lower_half_top_permille ? lower_half_top_permille :
                                  config->roi_top_permille;
    const int full_roi_bottom = config->roi_bottom_permille > 1000 ?
        1000 : config->roi_bottom_permille;
    if (full_roi_top >= full_roi_bottom) return result;

    camera_line_config_t cropped_config = *config;
    cropped_config.roi_top_permille =
        (full_roi_top - lower_half_top_permille) * 2;
    cropped_config.roi_bottom_permille =
        (full_roi_bottom - lower_half_top_permille) * 2;

    const size_t split_row = height / 2;
    const size_t cropped_height = height - split_row;
    const size_t source_row = rotate_180 ? 0 : split_row;
    const uint8_t *cropped_pixels =
        pixels + source_row * width * 3U;
    return camera_line_analyze_rgb888_internal(
        cropped_pixels, width, cropped_height, rotate_180,
        &cropped_config, workspace, has_previous_line,
        previous_center_permille, previous_steering_permille);
}
