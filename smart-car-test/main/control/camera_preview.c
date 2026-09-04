#include "camera_preview.h"

#include <string.h>

static const uint8_t CAMERA_PREVIEW_MAGIC[8] = {
    0xa5, 0x5a, 0xc3, 0x3c, 'S', 'C', 'V', '1',
};

_Static_assert(sizeof(camera_preview_packet_t) == 4836,
               "camera preview wire packet size changed");

static int clamp_int(int value, int minimum, int maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

static uint8_t rgb332(const uint8_t *rgb)
{
    return (uint8_t)((rgb[0] & 0xe0U) |
                     ((rgb[1] >> 3) & 0x1cU) |
                     (rgb[2] >> 6));
}

static uint8_t grayscale(const uint8_t *rgb)
{
    return (uint8_t)((77U * rgb[0] + 150U * rgb[1] + 29U * rgb[2]) >> 8);
}

uint32_t camera_preview_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffffU;
    if (data == NULL) return 0;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (unsigned bit = 0; bit < 8; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (0xedb88320U & mask);
        }
    }
    return ~crc;
}

static void draw_marker(camera_preview_packet_t *packet,
                        int center_x, int center_y, uint8_t color)
{
    for (int dy = -1; dy <= 1; ++dy) {
        const int y = center_y + dy;
        if (y < 0 || y >= packet->height) continue;
        for (int dx = -1; dx <= 1; ++dx) {
            const int x = center_x + dx;
            if (x >= 0 && x < packet->width) {
                packet->pixels[y * packet->width + x] = color;
            }
        }
    }
}

static void draw_box(camera_preview_packet_t *packet,
                     int left, int top, int right, int bottom,
                     uint8_t color)
{
    left = clamp_int(left, 0, packet->width - 1);
    right = clamp_int(right, left, packet->width - 1);
    top = clamp_int(top, 0, packet->height - 1);
    bottom = clamp_int(bottom, top, packet->height - 1);
    for (int x = left; x <= right; ++x) {
        packet->pixels[top * packet->width + x] = color;
        packet->pixels[bottom * packet->width + x] = color;
    }
    for (int y = top; y <= bottom; ++y) {
        packet->pixels[y * packet->width + left] = color;
        packet->pixels[y * packet->width + right] = color;
    }
}

static int center_to_pixel(int center_permille,
                           const camera_line_config_t *config,
                           int roi_left, int roi_width)
{
    const int unscaled =
        (center_permille - config->center_offset_permille) * 1000 /
        config->horizontal_scale_permille;
    const int local =
        (clamp_int(unscaled, -1000, 1000) + 1000) *
        (roi_width - 1) / 2000;
    return roi_left + local;
}

static int path_y_to_pixel(int y_permille, int roi_top, int roi_height)
{
    return roi_top + clamp_int(y_permille, 0, 1000) *
        (roi_height - 1) / 1000;
}

static int permille_to_pixel(int coordinate_permille, int extent)
{
    if (extent <= 1) return 0;
    return (clamp_int(coordinate_permille, 0, 1000) * (extent - 1) +
            500) / 1000;
}

static void draw_target(camera_preview_packet_t *packet,
                        const camera_ball_observation_t *target,
                        uint8_t candidate_color, uint8_t detected_color)
{
    if (target == NULL || !target->candidate) return;
    const uint8_t color = target->detected ?
        detected_color : candidate_color;
    draw_box(packet,
             permille_to_pixel(target->box_left_permille, packet->width),
             permille_to_pixel(target->box_top_permille, packet->height),
             permille_to_pixel(target->box_right_permille, packet->width),
             permille_to_pixel(target->box_bottom_permille, packet->height),
             color);
    const int center_x = clamp_int(
        ((int)target->center_x_permille + 1000) * packet->width / 2000,
        0, packet->width - 1);
    const int center_y = clamp_int(
        (int)target->center_y_permille * packet->height / 1000,
        0, packet->height - 1);
    /* The 3x3 marker makes a three-pixel far target visible after preview
     * downsampling. */
    draw_marker(packet, center_x, center_y, color);
}

bool camera_preview_build_rgb332(
    camera_preview_packet_t *packet,
    const uint8_t *rgb888_pixels, size_t width, size_t height,
    const camera_line_config_t *config,
    const camera_line_analysis_t *analysis,
    const camera_ball_observation_t *ball,
    const camera_ball_observation_t *left_target,
    const camera_ball_observation_t *right_target,
    uint32_t sequence, uint32_t timestamp_ms)
{
    if (packet == NULL || rgb888_pixels == NULL || config == NULL ||
        analysis == NULL || width < CAMERA_PREVIEW_WIDTH ||
        height < CAMERA_PREVIEW_HEIGHT ||
        config->horizontal_scale_permille <= 0) {
        return false;
    }

    memset(packet, 0, sizeof(*packet));
    memcpy(packet->magic, CAMERA_PREVIEW_MAGIC, sizeof(packet->magic));
    packet->version = CAMERA_PREVIEW_VERSION;
    packet->width = CAMERA_PREVIEW_WIDTH;
    packet->height = CAMERA_PREVIEW_HEIGHT;
    packet->pixel_format = CAMERA_PREVIEW_PIXEL_FORMAT_RGB332;
    packet->flags = (analysis->valid ? CAMERA_PREVIEW_FLAG_FRAME_VALID : 0) |
        (analysis->line_detected ? CAMERA_PREVIEW_FLAG_LINE_DETECTED : 0) |
        (analysis->finish_detected ? CAMERA_PREVIEW_FLAG_FINISH_DETECTED : 0) |
        (ball != NULL && ball->candidate ?
            CAMERA_PREVIEW_FLAG_BALL_CANDIDATE : 0) |
        (ball != NULL && ball->detected ?
            CAMERA_PREVIEW_FLAG_BALL_DETECTED : 0);
    packet->threshold = analysis->threshold;
    packet->contrast = analysis->contrast;
    packet->sequence = sequence;
    packet->timestamp_ms = timestamp_ms;
    packet->center_permille = analysis->center_permille;
    packet->far_center_permille = analysis->far_center_permille;
    packet->steering_permille = analysis->steering_permille;
    packet->payload_size = CAMERA_PREVIEW_PIXEL_COUNT;

    const size_t source_roi_left = width *
        (size_t)config->roi_left_permille / 1000U;
    const size_t source_roi_right = width *
        (size_t)config->roi_right_permille / 1000U;
    const size_t source_roi_top = height *
        (size_t)config->roi_top_permille / 1000U;
    const size_t source_roi_bottom = height *
        (size_t)config->roi_bottom_permille / 1000U;
    for (size_t preview_y = 0; preview_y < CAMERA_PREVIEW_HEIGHT;
         ++preview_y) {
        const size_t source_y_begin =
            preview_y * height / CAMERA_PREVIEW_HEIGHT;
        const size_t source_y_end =
            (preview_y + 1U) * height / CAMERA_PREVIEW_HEIGHT;
        for (size_t preview_x = 0; preview_x < CAMERA_PREVIEW_WIDTH;
             ++preview_x) {
            const size_t source_x_begin =
                preview_x * width / CAMERA_PREVIEW_WIDTH;
            const size_t source_x_end =
                (preview_x + 1U) * width / CAMERA_PREVIEW_WIDTH;
            uint32_t sum_red = 0;
            uint32_t sum_green = 0;
            uint32_t sum_blue = 0;
            uint32_t sample_count = 0;
            bool contains_black = false;
            for (size_t source_y = source_y_begin;
                 source_y < source_y_end; ++source_y) {
                for (size_t source_x = source_x_begin;
                     source_x < source_x_end; ++source_x) {
                    const uint8_t *rgb = rgb888_pixels +
                        (source_y * width + source_x) * 3U;
                    sum_red += rgb[0];
                    sum_green += rgb[1];
                    sum_blue += rgb[2];
                    ++sample_count;
                    if (analysis->valid &&
                        analysis->contrast >= config->minimum_contrast &&
                        source_x >= source_roi_left &&
                        source_x < source_roi_right &&
                        source_y >= source_roi_top &&
                        source_y < source_roi_bottom &&
                        grayscale(rgb) <= analysis->threshold) {
                        contains_black = true;
                    }
                }
            }
            const uint8_t averaged_rgb[3] = {
                (uint8_t)(sum_red / sample_count),
                (uint8_t)(sum_green / sample_count),
                (uint8_t)(sum_blue / sample_count),
            };
            const size_t preview_index =
                preview_y * CAMERA_PREVIEW_WIDTH + preview_x;
            packet->pixels[preview_index] = contains_black ?
                0xe0 : rgb332(averaged_rgb);
        }
    }

    const int roi_left = (int)(CAMERA_PREVIEW_WIDTH *
        (size_t)config->roi_left_permille / 1000U);
    const int roi_right = clamp_int((int)(CAMERA_PREVIEW_WIDTH *
        (size_t)config->roi_right_permille / 1000U), roi_left + 1,
        CAMERA_PREVIEW_WIDTH);
    const int roi_top = (int)(CAMERA_PREVIEW_HEIGHT *
        (size_t)config->roi_top_permille / 1000U);
    const int roi_bottom = clamp_int((int)(CAMERA_PREVIEW_HEIGHT *
        (size_t)config->roi_bottom_permille / 1000U), roi_top + 1,
        CAMERA_PREVIEW_HEIGHT);

    /* Yellow is the exact analysis ROI. */
    for (int x = roi_left; x < roi_right; ++x) {
        packet->pixels[roi_top * CAMERA_PREVIEW_WIDTH + x] = 0xfc;
        packet->pixels[(roi_bottom - 1) * CAMERA_PREVIEW_WIDTH + x] = 0xfc;
    }
    for (int y = roi_top; y < roi_bottom; ++y) {
        packet->pixels[y * CAMERA_PREVIEW_WIDTH + roi_left] = 0xfc;
        packet->pixels[y * CAMERA_PREVIEW_WIDTH + roi_right - 1] = 0xfc;
    }

    if (analysis->line_detected) {
        const int roi_width = roi_right - roi_left;
        const int near_x = center_to_pixel(
            analysis->center_permille, config, roi_left, roi_width);
        const int far_x = center_to_pixel(
            analysis->far_center_permille, config, roi_left, roi_width);
        const int roi_height = roi_bottom - roi_top;
        const int near_y = path_y_to_pixel(
            analysis->center_y_permille, roi_top, roi_height);
        const int far_y = path_y_to_pixel(
            analysis->far_center_y_permille, roi_top, roi_height);
        draw_marker(packet, near_x, near_y, 0x1c); /* green */
        draw_marker(packet, far_x, far_y, 0x03);   /* blue */
    }

    if (ball != NULL && ball->candidate) {
        /* The ball detector operates on the full frame.  Its orange/magenta
         * box is therefore deliberately not clipped to the yellow line ROI. */
        const uint8_t color = ball->color == BALL_COLOR_GREEN ?
            (ball->detected ? 0xff : 0xb6) :
            (ball->detected ? 0xe3 : 0xf0);
        draw_box(packet,
                 permille_to_pixel(ball->box_left_permille, packet->width),
                 permille_to_pixel(ball->box_top_permille, packet->height),
                 permille_to_pixel(ball->box_right_permille, packet->width),
                 permille_to_pixel(ball->box_bottom_permille, packet->height),
                 color);
        const int center_x = clamp_int(
            ((int)ball->center_x_permille + 1000) * packet->width / 2000,
            0, packet->width - 1);
        const int center_y = clamp_int(
            (int)ball->center_y_permille * packet->height / 1000,
            0, packet->height - 1);
        draw_marker(packet, center_x, center_y, color);
    }

    /* Cyan is left_target (the delivery goal); green-cyan is right_target.
     * Both are full-frame overlays and remain visible when only a few source
     * pixels survive at long range. */
    draw_target(packet, left_target, 0x0f, 0x1f);
    draw_target(packet, right_target, 0x0d, 0x1d);

    packet->payload_crc32 = camera_preview_crc32(
        packet->pixels, packet->payload_size);
    return true;
}
