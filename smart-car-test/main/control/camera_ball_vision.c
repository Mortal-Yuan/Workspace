#include "camera_ball_vision.h"

#include <limits.h>

enum {
    BALL_PIXEL_WEAK_COLOR = 1U << 0,
    BALL_PIXEL_STRONG_COLOR = 1U << 1,
    BALL_PIXEL_SUPPORT = 1U << 2,
    BALL_PIXEL_PENDING = 1U << 3,
    BALL_PIXEL_VISITED = 1U << 4,
    BALL_PIXEL_HIGHLIGHT = 1U << 5,
    BALL_PIXEL_BLUE = 1U << 6,
};

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static uint16_t coordinate_permille(size_t coordinate, size_t extent)
{
    if (extent <= 1U) return 0;
    return (uint16_t)((coordinate * 1000U + (extent - 1U) / 2U) /
                      (extent - 1U));
}

static size_t scale_reference_pixels(size_t pixels, size_t extent,
                                     size_t reference_extent)
{
    return (pixels * extent + reference_extent - 1U) / reference_extent;
}

static int maximum_channel(const uint8_t *rgb)
{
    int maximum = rgb[0] > rgb[1] ? rgb[0] : rgb[1];
    return maximum > rgb[2] ? maximum : rgb[2];
}

static int minimum_channel(const uint8_t *rgb)
{
    int minimum = rgb[0] < rgb[1] ? rgb[0] : rgb[1];
    return minimum < rgb[2] ? minimum : rgb[2];
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

static int target_channel(ball_color_t color)
{
    return color == BALL_COLOR_BLUE ? 2 : 0;
}

static int target_color_score(const uint8_t *rgb, ball_color_t color)
{
    const int target = target_channel(color);
    const int other_a = target == 0 ? 1 : 0;
    const int other_b = target == 2 ? 1 : 2;
    return 2 * (int)rgb[target] - rgb[other_a] - rgb[other_b];
}

static bool color_gate(const uint8_t *rgb, ball_color_t color,
                       int minimum, int dominance, int ratio_permille)
{
    const int target = target_channel(color);
    const int other_a = target == 0 ? 1 : 0;
    const int other_b = target == 2 ? 1 : 2;
    const int competitor = rgb[other_a] > rgb[other_b] ?
        rgb[other_a] : rgb[other_b];
    const int total = rgb[0] + rgb[1] + rgb[2];
    return rgb[target] >= minimum &&
        rgb[target] - competitor >= dominance && total > 0 &&
        rgb[target] * 1000 >= ratio_permille * total;
}

static bool is_weak_color(const uint8_t *rgb, ball_color_t color,
                          const camera_ball_config_t *config)
{
    if (color == BALL_COLOR_BLUE) {
        return color_gate(rgb, color, config->blue_minimum,
                          config->blue_dominance,
                          config->blue_ratio_permille);
    }
    return color_gate(rgb, color, config->red_minimum,
                      config->red_dominance,
                      config->red_ratio_permille);
}

static bool is_strong_color(const uint8_t *rgb, ball_color_t color,
                            const camera_ball_config_t *config)
{
    if (color == BALL_COLOR_BLUE) {
        return color_gate(rgb, color, config->blue_minimum,
                          config->strong_blue_dominance,
                          config->strong_blue_ratio_permille);
    }
    return color_gate(rgb, color, config->red_minimum,
                      config->strong_red_dominance,
                      config->strong_red_ratio_permille);
}

static bool is_highlight_support(const uint8_t *rgb,
                                 ball_color_t color,
                                 const camera_ball_config_t *config)
{
    const int maximum = maximum_channel(rgb);
    const int target = rgb[target_channel(color)];
    return maximum >= config->highlight_minimum &&
        maximum - minimum_channel(rgb) <= config->highlight_max_chroma &&
        target + config->highlight_target_tolerance >= maximum;
}

/* Add only bright, nearly neutral pixels that are locally surrounded by one
 * target color. Pending pixels become support after the scan, so a configured
 * pass grows by at most one pixel and cannot flood a white scene. */
static void repair_specular_highlights(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config, uint8_t *mask)
{
    /* Keep this pass count bounded at native resolution. Scaling the old
     * 80x60 two-pixel radius to 16 VGA passes made a bright background grow
     * into a target component and held the decode task for several seconds. */
    for (int pass = 0; pass < config->highlight_expand_passes; ++pass) {
        size_t additions = 0;
        for (size_t y = 1; y + 1U < height; ++y) {
            for (size_t x = 1; x + 1U < width; ++x) {
                const size_t index = y * width + x;
                if ((mask[index] & BALL_PIXEL_SUPPORT) != 0U) {
                    continue;
                }
                unsigned red_neighbours = 0;
                unsigned blue_neighbours = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const size_t neighbour =
                            (size_t)((int)y + dy) * width +
                            (size_t)((int)x + dx);
                        if ((mask[neighbour] & BALL_PIXEL_SUPPORT) != 0U) {
                            if ((mask[neighbour] & BALL_PIXEL_BLUE) != 0U) {
                                ++blue_neighbours;
                            } else {
                                ++red_neighbours;
                            }
                        }
                    }
                }
                ball_color_t color = BALL_COLOR_NONE;
                if (red_neighbours >= 3U &&
                    red_neighbours > blue_neighbours) {
                    color = BALL_COLOR_RED;
                } else if (blue_neighbours >= 3U &&
                           blue_neighbours > red_neighbours) {
                    color = BALL_COLOR_BLUE;
                }
                if (color != BALL_COLOR_NONE && is_highlight_support(
                        pixel_at(pixels, width, height, x, y, rotate_180),
                        color, config)) {
                    mask[index] |= BALL_PIXEL_PENDING |
                        (color == BALL_COLOR_BLUE ? BALL_PIXEL_BLUE : 0U);
                    ++additions;
                }
            }
        }
        if (additions == 0U) break;
        for (size_t index = 0; index < width * height; ++index) {
            if ((mask[index] & BALL_PIXEL_PENDING) != 0U) {
                mask[index] = (uint8_t)(
                    (mask[index] & ~BALL_PIXEL_PENDING) |
                    BALL_PIXEL_SUPPORT | BALL_PIXEL_HIGHLIGHT);
            }
        }
    }
}

static uint16_t component_confidence(
    int mean_color_dominance, int strong_color_dominance,
    uint16_t strong_ratio_permille,
    uint16_t area_permille, uint16_t fill_permille,
    uint16_t roundness_permille, const camera_ball_config_t *config)
{
    const int color_score = clamp_int(
        mean_color_dominance * 1000 / strong_color_dominance, 0, 1000);
    const int seed_score = clamp_int(
        strong_ratio_permille * 1000 /
            (config->minimum_strong_ratio_permille * 2), 0, 1000);
    const int area_score = clamp_int(
        area_permille * 1000 / (config->minimum_area_permille * 4),
        0, 1000);
    const int fill_score = clamp_int(
        fill_permille * 1000 / (config->minimum_fill_permille * 2),
        0, 1000);
    const int roundness_score = clamp_int(
        roundness_permille * 1000 / config->minimum_roundness_permille,
        0, 1000);
    return (uint16_t)((3 * color_score + 3 * seed_score + area_score +
                       fill_score + 2 * roundness_score) / 10);
}

static camera_ball_observation_t analyze_target_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    ball_color_t requested_color, int minimum_center_x_permille,
    int maximum_center_x_permille,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace)
{
    camera_ball_observation_t result = {0};
    if (pixels == NULL || config == NULL || workspace == NULL ||
        width == 0 || height == 0 ||
        width > CAMERA_BALL_VISION_MAX_WIDTH ||
        height > CAMERA_BALL_VISION_MAX_HEIGHT ||
        (requested_color != BALL_COLOR_NONE &&
         requested_color != BALL_COLOR_RED &&
         requested_color != BALL_COLOR_BLUE)) {
        return result;
    }
    result.valid = true;

    const size_t pixel_count = width * height;
    const size_t reference_pixel_count =
        CAMERA_BALL_VISION_REFERENCE_WIDTH *
        CAMERA_BALL_VISION_REFERENCE_HEIGHT;
    const uint32_t minimum_strong_pixels = (uint32_t)(
        ((uint64_t)config->minimum_strong_pixels * pixel_count +
         reference_pixel_count - 1U) / reference_pixel_count);
    const uint32_t blue_target_minimum_pixels = (uint32_t)(
        ((uint64_t)config->blue_target_minimum_pixels * pixel_count +
         reference_pixel_count - 1U) / reference_pixel_count);
    const uint32_t blue_target_minimum_strong_pixels = (uint32_t)(
        ((uint64_t)config->blue_target_minimum_strong_pixels * pixel_count +
         reference_pixel_count - 1U) / reference_pixel_count);
    const size_t horizontal_edge_margin = scale_reference_pixels(
        (size_t)config->edge_margin_pixels, width,
        CAMERA_BALL_VISION_REFERENCE_WIDTH);
    const size_t vertical_edge_margin = scale_reference_pixels(
        (size_t)config->edge_margin_pixels, height,
        CAMERA_BALL_VISION_REFERENCE_HEIGHT);
    uint8_t *mask = workspace->mask;
    int best_red_score = INT_MIN;
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            const size_t index = y * width + x;
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
            mask[index] = 0;
            const bool weak_red = requested_color != BALL_COLOR_BLUE &&
                is_weak_color(rgb, BALL_COLOR_RED, config);
            const bool weak_blue = requested_color != BALL_COLOR_RED &&
                is_weak_color(rgb, BALL_COLOR_BLUE, config);
            if (weak_red || weak_blue) {
                const ball_color_t color = weak_blue &&
                    (!weak_red || target_color_score(rgb, BALL_COLOR_BLUE) >
                                  target_color_score(rgb, BALL_COLOR_RED)) ?
                    BALL_COLOR_BLUE : BALL_COLOR_RED;
                mask[index] = BALL_PIXEL_WEAK_COLOR | BALL_PIXEL_SUPPORT |
                    (color == BALL_COLOR_BLUE ? BALL_PIXEL_BLUE : 0U);
                if (is_strong_color(rgb, color, config)) {
                    mask[index] |= BALL_PIXEL_STRONG_COLOR;
                }
            }
        }
    }
    repair_specular_highlights(
        pixels, width, height, rotate_180, config, mask);

    uint32_t best_area = 0;
    uint32_t best_red_area = 0;
    uint32_t best_strong_area = 0;
    uint32_t best_highlight_area = 0;
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
    uint16_t best_confidence = 0;
    ball_color_t best_color = BALL_COLOR_NONE;
    bool best_is_ball = false;
    bool best_is_far_ball = false;

    for (size_t start = 0; start < pixel_count; ++start) {
        if ((mask[start] & BALL_PIXEL_SUPPORT) == 0U ||
            (mask[start] & BALL_PIXEL_VISITED) != 0U) {
            continue;
        }
        if (result.component_count < UINT8_MAX) {
            ++result.component_count;
        }
        const uint8_t component_color_flag =
            mask[start] & BALL_PIXEL_BLUE;
        const ball_color_t component_color = component_color_flag != 0U ?
            BALL_COLOR_BLUE : BALL_COLOR_RED;

        uint32_t head = 0;
        uint32_t tail = 0;
        workspace->queue[tail++] = (uint16_t)start;
        mask[start] |= BALL_PIXEL_VISITED;
        uint32_t area = 0;
        uint32_t red_area = 0;
        uint32_t strong_area = 0;
        uint32_t highlight_area = 0;
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
            const uint8_t flags = mask[index];
            ++area;
            sum_x += (uint32_t)x;
            sum_y += (uint32_t)y;
            if ((flags & BALL_PIXEL_WEAK_COLOR) != 0U) {
                const uint8_t *rgb = pixel_at(
                    pixels, width, height, x, y, rotate_180);
                ++red_area;
                sum_r += rgb[0];
                sum_g += rgb[1];
                sum_b += rgb[2];
            }
            if ((flags & BALL_PIXEL_STRONG_COLOR) != 0U) ++strong_area;
            if ((flags & BALL_PIXEL_HIGHLIGHT) != 0U) ++highlight_area;
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
                    if ((mask[next] & BALL_PIXEL_SUPPORT) != 0U &&
                        (mask[next] & BALL_PIXEL_VISITED) == 0U &&
                        (mask[next] & BALL_PIXEL_BLUE) ==
                            component_color_flag) {
                        mask[next] |= BALL_PIXEL_VISITED;
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
        const int mean_red = red_area > 0U ? (int)(sum_r / red_area) : 0;
        const int mean_green = red_area > 0U ? (int)(sum_g / red_area) : 0;
        const int mean_blue = red_area > 0U ? (int)(sum_b / red_area) : 0;
        const int mean_target = component_color == BALL_COLOR_BLUE ?
            mean_blue : mean_red;
        const int mean_other_a = component_color == BALL_COLOR_BLUE ?
            mean_red : mean_green;
        const int mean_other_b = component_color == BALL_COLOR_BLUE ?
            mean_green : mean_blue;
        const int mean_competitor = mean_other_a > mean_other_b ?
            mean_other_a : mean_other_b;
        const int mean_color_dominance = mean_target - mean_competitor;
        const int strong_color_dominance =
            component_color == BALL_COLOR_BLUE ?
                config->strong_blue_dominance :
                config->strong_red_dominance;
        const int minimum_mean_color_dominance =
            component_color == BALL_COLOR_BLUE ?
                config->minimum_mean_blue_dominance :
                config->minimum_mean_red_dominance;
        const int minimum_roundness =
            component_color == BALL_COLOR_BLUE ?
                config->minimum_blue_roundness_permille :
                config->minimum_roundness_permille;
        const uint16_t strong_ratio_permille = red_area > 0U ?
            (uint16_t)((uint64_t)strong_area * 1000U / red_area) : 0U;
        const uint16_t confidence = component_confidence(
            mean_color_dominance, strong_color_dominance,
            strong_ratio_permille, area_permille, fill_permille,
            roundness_permille, config);
        const int component_center_x = clamp_int(
            (int)((uint64_t)sum_x * 2000U / (area * width)) - 1000,
            -1000, 1000);
        if (component_center_x < minimum_center_x_permille ||
            component_center_x > maximum_center_x_permille) {
            continue;
        }
        const bool common_ball_shape =
            fill_permille >= config->minimum_fill_permille &&
            roundness_permille >= minimum_roundness &&
            strong_area >= minimum_strong_pixels &&
            strong_ratio_permille >=
                config->minimum_strong_ratio_permille &&
            mean_color_dominance >= minimum_mean_color_dominance &&
            minimum_x >= horizontal_edge_margin &&
            minimum_y >= vertical_edge_margin &&
            maximum_x + horizontal_edge_margin < width &&
            maximum_y + vertical_edge_margin < height;
        const bool normal_ball_size =
            area_permille >= config->minimum_area_permille;
        const bool far_ball_size =
            area_permille >= config->far_minimum_area_permille &&
            area_permille < config->minimum_area_permille &&
            confidence >= config->far_minimum_confidence_permille;
        /* A blue destination is a rectangular field marker, not a ball.  At
         * long range it can shrink below one permille or touch a side edge,
         * so use a raw-pixel gate and do not require the ball edge margin.
         * Color strength, fill, shape, confidence, and multi-frame tracking
         * still reject isolated blue noise. */
        const bool normal_blue_target_shape =
            component_color == BALL_COLOR_BLUE &&
            (normal_ball_size || far_ball_size) &&
            strong_area >= minimum_strong_pixels &&
            strong_ratio_permille >=
                config->minimum_strong_ratio_permille &&
            mean_color_dominance >= minimum_mean_color_dominance &&
            fill_permille >= config->minimum_fill_permille &&
            roundness_permille >= config->minimum_blue_roundness_permille;
        const bool tiny_blue_target_shape =
            component_color == BALL_COLOR_BLUE && !normal_ball_size &&
            red_area >= blue_target_minimum_pixels &&
            strong_area >= blue_target_minimum_strong_pixels &&
            strong_ratio_permille >=
                config->minimum_strong_ratio_permille &&
            mean_color_dominance >=
                config->blue_target_minimum_mean_dominance &&
            fill_permille >= config->minimum_fill_permille &&
            roundness_permille >=
                config->blue_target_minimum_roundness_permille &&
            confidence >=
                config->blue_target_minimum_confidence_permille;
        const bool ball_shape = component_color == BALL_COLOR_BLUE ?
            normal_blue_target_shape || tiny_blue_target_shape :
            common_ball_shape && (normal_ball_size || far_ball_size);
        const bool replace_best = best_area == 0U ||
            (ball_shape && !best_is_ball) ||
            (ball_shape == best_is_ball &&
             (confidence > best_confidence ||
              (confidence == best_confidence && area > best_area)));
        if (!replace_best) continue;

        best_area = area;
        best_red_area = red_area;
        best_strong_area = strong_area;
        best_highlight_area = highlight_area;
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
        best_confidence = confidence;
        best_color = component_color;
        best_is_far_ball = ball_shape &&
            (far_ball_size ||
             (component_color == BALL_COLOR_BLUE && !normal_ball_size));
    }

    if (best_area == 0U || best_red_area == 0U) return result;

    const size_t box_width = best_maximum_x - best_minimum_x + 1U;
    const size_t box_height = best_maximum_y - best_minimum_y + 1U;
    const int center_x = (int)((uint64_t)best_sum_x * 2000U /
                               (best_area * width)) - 1000;
    result.candidate = best_is_ball;
    result.far_candidate = best_is_far_ball;
    result.color = best_color;
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
    result.confidence_permille = best_confidence;
    result.mean_red = (uint8_t)(best_sum_r / best_red_area);
    result.mean_green = (uint8_t)(best_sum_g / best_red_area);
    result.mean_blue = (uint8_t)(best_sum_b / best_red_area);
    result.matched_pixels = best_red_area > UINT16_MAX ?
        UINT16_MAX : (uint16_t)best_red_area;
    result.strong_pixels = best_strong_area > UINT16_MAX ?
        UINT16_MAX : (uint16_t)best_strong_area;
    result.highlight_pixels = best_highlight_area > UINT16_MAX ?
        UINT16_MAX : (uint16_t)best_highlight_area;
    result.box_left_permille = coordinate_permille(best_minimum_x, width);
    result.box_top_permille = coordinate_permille(best_minimum_y, height);
    result.box_right_permille = coordinate_permille(best_maximum_x, width);
    result.box_bottom_permille = coordinate_permille(best_maximum_y, height);
    return result;
}

camera_ball_observation_t camera_ball_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace)
{
    return analyze_target_rgb888(
        pixels, width, height, rotate_180, BALL_COLOR_NONE, -1000, 1000,
        config, workspace);
}

camera_ball_observation_t camera_ball_analyze_color_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    ball_color_t requested_color, const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace)
{
    return analyze_target_rgb888(
        pixels, width, height, rotate_180, requested_color, -1000, 1000,
        config, workspace);
}

camera_blue_target_pair_t camera_blue_targets_analyze_rgb888(
    const uint8_t *pixels, size_t width, size_t height, bool rotate_180,
    const camera_ball_config_t *config,
    camera_ball_vision_workspace_t *workspace)
{
    camera_blue_target_pair_t targets = {0};
    targets.left_target = analyze_target_rgb888(
        pixels, width, height, rotate_180, BALL_COLOR_BLUE, -1000, -1,
        config, workspace);
    targets.right_target = analyze_target_rgb888(
        pixels, width, height, rotate_180, BALL_COLOR_BLUE, 0, 1000,
        config, workspace);
    return targets;
}
