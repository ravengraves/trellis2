#include "ros3.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    IMAGE_VIEW_TOPIC_BYTES = 256,
    IMAGE_VIEW_BBOX_NAME_BYTES = 64,
    IMAGE_VIEW_INITIAL_BOX_CAPACITY = 64,
    IMAGE_VIEW_BBOX_BUFFER_BYTES = 256 * 1024,
    IMAGE_VIEW_MAX_IMAGE_RANK = 3,
    IMAGE_VIEW_DEFAULT_BUFFER_BYTES = 8 * 1024 * 1024,
    IMAGE_VIEW_CONNECT_RETRY_NS = 250000000ULL,
    IMAGE_VIEW_IDLE_SLEEP_NS = 10000000ULL,
    IMAGE_VIEW_WAIT_BUDGET_NS = 16000000ULL,
    IMAGE_VIEW_STATUS_STARTING = 0,
    IMAGE_VIEW_STATUS_WAITING_FOR_SUBSCRIBER = 1,
    IMAGE_VIEW_STATUS_WAITING_FOR_FRAME = 2,
    IMAGE_VIEW_STATUS_FRAME_READY = 3,
    IMAGE_VIEW_STATUS_READ_ERROR = -3,
    IMAGE_VIEW_STATUS_NOT_NUMERIC_ARRAY = -4,
    IMAGE_VIEW_STATUS_UNSUPPORTED_TYPE = -5,
    IMAGE_VIEW_STATUS_BAD_SHAPE = -6,
    IMAGE_VIEW_STATUS_PAYLOAD_READ_ERROR = -7,
    IMAGE_VIEW_STATUS_BUFFER_TOO_SMALL = -8
};

typedef enum {
    IMAGE_LAYOUT_FLAT = 0,
    IMAGE_LAYOUT_GRAY_2D = 1,
    IMAGE_LAYOUT_HWC = 2,
    IMAGE_LAYOUT_CHW = 3
} image_layout;

typedef struct {
    char input_topic[IMAGE_VIEW_TOPIC_BYTES];
    char bbox_topic[IMAGE_VIEW_TOPIC_BYTES];
    int width;
    int height;
    int channels;
    uint64_t input_buffer_size;
    int latest_only;
} image_view_options;

typedef struct {
    int width;
    int height;
    int channels;
    int64_t element_count;
    int64_t byte_count;
    image_layout layout;
    mirage_array_type type;
} image_spec;

typedef struct {
    double xmin;
    double ymin;
    double xmax;
    double ymax;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    char name[IMAGE_VIEW_BBOX_NAME_BYTES];
    int name_len;
    int valid;
} image_view_bbox;

static image_view_options g_options;
static rose_node *g_node = NULL;
static rose_subscriber *g_sub = NULL;
static rose_subscriber *g_bbox_sub = NULL;
static mirage_msg *g_msg = NULL;
static mirage_msg *g_bbox_msg = NULL;
static uint8_t *g_raw = NULL;
static uint64_t g_seq = 0;
static uint64_t g_last_retry_ns = 0;
static uint64_t g_bbox_last_retry_ns = 0;
static int g_payload_size = 0;
static int g_status = IMAGE_VIEW_STATUS_STARTING;
static image_view_bbox *g_bboxes = NULL;
static int g_bbox_capacity = 0;
static image_view_bbox *g_bbox_scratch = NULL;
static int g_bbox_scratch_capacity = 0;
static int g_bbox_count = 0;

void image_view_bridge_shutdown(void);

static int parse_int_arg(const char *text, int *out_value) {
    char *end = NULL;
    long value = 0;
    if (!text || !out_value) {
        return 0;
    }
    value = strtol(text, &end, 10);
    if (end == text || (end && *end != '\0')) {
        return 0;
    }
    if (value < INT32_MIN || value > INT32_MAX) {
        return 0;
    }
    *out_value = (int)value;
    return 1;
}

static int parse_u64_arg(const char *text, uint64_t *out_value) {
    char *end = NULL;
    unsigned long long value = 0;
    if (!text || !out_value) {
        return 0;
    }
    value = strtoull(text, &end, 10);
    if (end == text || (end && *end != '\0')) {
        return 0;
    }
    *out_value = (uint64_t)value;
    return 1;
}

static int parse_bool_flag_arg(const char *text, int *out_value) {
    int parsed = 0;
    if (!parse_int_arg(text, &parsed)) {
        return 0;
    }
    if (parsed != 0 && parsed != 1) {
        return 0;
    }
    *out_value = parsed;
    return 1;
}

static int checked_multiply_i64(int64_t a, int64_t b, int64_t *out_value) {
    if (!out_value || a < 0 || b < 0) {
        return 0;
    }
    if (a == 0 || b == 0) {
        *out_value = 0;
        return 1;
    }
    if (a > INT64_MAX / b) {
        return 0;
    }
    *out_value = a * b;
    return 1;
}

static int is_channel_dimension(int64_t value) {
    return value == 1 || value == 3 || value == 4;
}

static int is_supported_image_type(mirage_array_type type) {
    return type == MIRAGE_ARRAY_U8 || type == MIRAGE_ARRAY_U16 ||
           type == MIRAGE_ARRAY_F32 || type == MIRAGE_ARRAY_F64;
}

typedef struct {
    int width;
    int height;
} image_resolution;

static int match_known_resolution(int64_t pixels, int *out_w, int *out_h) {
    static const image_resolution kKnown[] = {
        {1920, 1080}, {1600, 1200}, {1280, 960}, {1280, 800},
        {1280, 720},  {1024, 768},  {960, 540},  {848, 480},
        {800, 600},   {640, 480},   {640, 360},  {424, 240},
        {400, 300},   {384, 288},   {320, 240},  {320, 180},
        {256, 144},   {160, 120}
    };
    size_t i = 0;
    if (!out_w || !out_h || pixels <= 0) {
        return 0;
    }
    for (i = 0; i < (sizeof(kKnown) / sizeof(kKnown[0])); ++i) {
        if ((int64_t)kKnown[i].width * (int64_t)kKnown[i].height == pixels) {
            *out_w = kKnown[i].width;
            *out_h = kKnown[i].height;
            return 1;
        }
    }
    return 0;
}

static int infer_factor_resolution(int64_t pixels, int *out_w, int *out_h) {
    static const double kAspectPref[] = {
        16.0 / 9.0, 4.0 / 3.0, 1.0, 16.0 / 10.0, 3.0 / 2.0, 5.0 / 4.0
    };
    int best_w = 0;
    int best_h = 0;
    double best_score = 1e30;
    int64_t h = 0;

    if (!out_w || !out_h || pixels <= 0) {
        return 0;
    }

    for (h = 1; h * h <= pixels; ++h) {
        int64_t w = 0;
        int64_t cand_w = 0;
        int64_t cand_h = 0;
        size_t ai = 0;
        if (pixels % h != 0) {
            continue;
        }
        w = pixels / h;
        cand_w = w;
        cand_h = h;
        if (cand_w < cand_h) {
            int64_t tmp = cand_w;
            cand_w = cand_h;
            cand_h = tmp;
        }
        if (cand_w <= 0 || cand_h <= 0) {
            continue;
        }

        {
            double aspect = (double)cand_w / (double)cand_h;
            double aspect_err = 1000.0;
            double score = 0.0;
            for (ai = 0; ai < (sizeof(kAspectPref) / sizeof(kAspectPref[0])); ++ai) {
                double err = fabs(aspect - kAspectPref[ai]);
                if (err < aspect_err) {
                    aspect_err = err;
                }
            }
            score = aspect_err * 1000.0;
            if ((cand_w % 2) != 0 || (cand_h % 2) != 0) {
                score += 50.0;
            }
            if ((cand_w % 8) != 0 || (cand_h % 2) != 0) {
                score += 10.0;
            }
            if (cand_w < cand_h) {
                score += 100.0;
            }
            if (score < best_score) {
                best_score = score;
                best_w = (int)cand_w;
                best_h = (int)cand_h;
            }
        }
    }

    if (best_w <= 0 || best_h <= 0) {
        return 0;
    }
    *out_w = best_w;
    *out_h = best_h;
    return 1;
}

static int infer_flat_image_spec(const image_view_options *options,
                                 int64_t elements,
                                 image_spec *spec) {
    static const int kAutoChannels[] = {3, 1, 4};
    int widths[4] = {0, 0, 0, 0};
    int heights[4] = {0, 0, 0, 0};
    int channels[4] = {0, 0, 0, 0};
    int candidate_count = 0;
    int ci = 0;

    if (!options || !spec || elements <= 0) {
        return 0;
    }

    if (options->channels > 0) {
        channels[candidate_count++] = options->channels;
    } else {
        size_t i = 0;
        for (i = 0; i < (sizeof(kAutoChannels) / sizeof(kAutoChannels[0])); ++i) {
            channels[candidate_count++] = kAutoChannels[i];
        }
    }

    for (ci = 0; ci < candidate_count; ++ci) {
        int ch = channels[ci];
        int64_t pixels = 0;
        int w = 0;
        int h = 0;

        if (ch <= 0 || elements % ch != 0) {
            continue;
        }
        pixels = elements / ch;
        if (options->width > 0 && options->height > 0) {
            if ((int64_t)options->width * (int64_t)options->height != pixels) {
                continue;
            }
            w = options->width;
            h = options->height;
        } else if (options->width > 0) {
            if (pixels % options->width != 0) {
                continue;
            }
            w = options->width;
            h = (int)(pixels / options->width);
        } else if (options->height > 0) {
            if (pixels % options->height != 0) {
                continue;
            }
            h = options->height;
            w = (int)(pixels / options->height);
        } else if (!match_known_resolution(pixels, &w, &h) &&
                   !infer_factor_resolution(pixels, &w, &h)) {
            continue;
        }

        if (w > 0 && h > 0) {
            spec->width = w;
            spec->height = h;
            spec->channels = ch;
            spec->element_count = elements;
            spec->layout = IMAGE_LAYOUT_FLAT;
            return 1;
        }
    }
    return 0;
}

static int resolve_image_spec(const image_view_options *options,
                              mirage_array_type type,
                              int64_t rank,
                              const int64_t *dims,
                              image_spec *out_spec) {
    image_spec spec;
    int64_t plane = 0;
    int64_t bytes_per_element = 0;

    if (!options || !dims || !out_spec || !is_supported_image_type(type)) {
        return 0;
    }

    memset(&spec, 0, sizeof(spec));
    spec.type = type;

    if (rank == 1) {
        if (!infer_flat_image_spec(options, dims[0], &spec)) {
            return 0;
        }
    } else if (rank == 2) {
        if (dims[0] <= 0 || dims[1] <= 0) {
            return 0;
        }
        spec.width = (int)dims[1];
        spec.height = (int)dims[0];
        spec.channels = 1;
        spec.layout = IMAGE_LAYOUT_GRAY_2D;
        if (!checked_multiply_i64(dims[0], dims[1], &spec.element_count)) {
            return 0;
        }
    } else if (rank == 3) {
        if (dims[0] <= 0 || dims[1] <= 0 || dims[2] <= 0) {
            return 0;
        }
        if (is_channel_dimension(dims[2])) {
            spec.height = (int)dims[0];
            spec.width = (int)dims[1];
            spec.channels = (int)dims[2];
            spec.layout = IMAGE_LAYOUT_HWC;
        } else if (is_channel_dimension(dims[0])) {
            spec.channels = (int)dims[0];
            spec.height = (int)dims[1];
            spec.width = (int)dims[2];
            spec.layout = IMAGE_LAYOUT_CHW;
        } else {
            return 0;
        }

        if (!checked_multiply_i64((int64_t)spec.width, (int64_t)spec.height, &plane)) {
            return 0;
        }
        if (!checked_multiply_i64(plane, (int64_t)spec.channels, &spec.element_count)) {
            return 0;
        }
    } else {
        return 0;
    }

    if (spec.width <= 0 || spec.height <= 0 || spec.channels <= 0) {
        return 0;
    }

    bytes_per_element = (int64_t)mirage_array_type_size[type];
    if (!checked_multiply_i64(spec.element_count, bytes_per_element, &spec.byte_count)) {
        return 0;
    }

    *out_spec = spec;
    return 1;
}

static uint8_t normalize_u8_sample(uint8_t value) {
    return value;
}

static uint8_t normalize_u16_sample(uint16_t value) {
    return (value > 255u) ? (uint8_t)(value / 257u) : (uint8_t)value;
}

static uint8_t normalize_float_sample(double value) {
    if (isfinite(value) && value >= 0.0 && value <= 1.0) {
        value *= 255.0;
    }
    if (!isfinite(value) || value <= 0.0) {
        return 0u;
    }
    if (value >= 255.0) {
        return 255u;
    }
    return (uint8_t)lround(value);
}

static uint8_t normalize_float_sample_by_max(double value, double max_value) {
    if (!isfinite(value) || !isfinite(max_value) || max_value <= 0.0 || value <= 0.0) {
        return 0u;
    }
    value = (value / max_value) * 255.0;
    if (value >= 255.0) {
        return 255u;
    }
    return (uint8_t)lround(value);
}

#define IMAGE_VIEW_CONVERT_IMPL(NAME, TYPE, NORMALIZE_EXPR)                                  \
    static void NAME(const TYPE *src, const image_spec *spec, uint8_t *dst_rgb) {           \
        int64_t pixels = (int64_t)spec->width * (int64_t)spec->height;                       \
        int64_t i;                                                                           \
        TYPE sample = (TYPE)0;                                                               \
        if (spec->channels == 1) {                                                           \
            for (i = 0; i < pixels; ++i) {                                                   \
                sample = src[i];                                                             \
                uint8_t v = (NORMALIZE_EXPR);                                                \
                dst_rgb[i * 3 + 0] = v;                                                      \
                dst_rgb[i * 3 + 1] = v;                                                      \
                dst_rgb[i * 3 + 2] = v;                                                      \
            }                                                                                \
            return;                                                                          \
        }                                                                                    \
        if (spec->layout == IMAGE_LAYOUT_CHW) {                                              \
            int64_t plane = pixels;                                                          \
            for (i = 0; i < pixels; ++i) {                                                   \
                uint8_t r = normalize_u8_sample(0u);                                         \
                uint8_t g = normalize_u8_sample(0u);                                         \
                uint8_t b = normalize_u8_sample(0u);                                         \
                sample = src[i];                                                             \
                r = (NORMALIZE_EXPR);                                                        \
                sample = src[plane + i];                                                     \
                g = (spec->channels > 1) ? (NORMALIZE_EXPR) : r;                             \
                sample = src[(plane * 2) + i];                                               \
                b = (spec->channels > 2) ? (NORMALIZE_EXPR) : 0u;                            \
                dst_rgb[i * 3 + 0] = r;                                                      \
                dst_rgb[i * 3 + 1] = g;                                                      \
                dst_rgb[i * 3 + 2] = b;                                                      \
            }                                                                                \
            return;                                                                          \
        }                                                                                    \
        for (i = 0; i < pixels; ++i) {                                                       \
            int64_t base = i * (int64_t)spec->channels;                                      \
            TYPE sample = src[base + 0];                                                     \
            uint8_t r = (NORMALIZE_EXPR);                                                    \
            sample = src[base + ((spec->channels > 1) ? 1 : 0)];                             \
            uint8_t g = (spec->channels > 1) ? (NORMALIZE_EXPR) : r;                         \
            sample = src[base + ((spec->channels > 2) ? 2 : 0)];                             \
            uint8_t b = (spec->channels > 2) ? (NORMALIZE_EXPR) : 0u;                        \
            dst_rgb[i * 3 + 0] = r;                                                          \
            dst_rgb[i * 3 + 1] = g;                                                          \
            dst_rgb[i * 3 + 2] = b;                                                          \
        }                                                                                    \
    }

IMAGE_VIEW_CONVERT_IMPL(image_view_convert_u8, uint8_t, normalize_u8_sample(sample))
IMAGE_VIEW_CONVERT_IMPL(image_view_convert_u16, uint16_t, normalize_u16_sample(sample))

static void image_view_convert_f32(const float *src,
                                   const image_spec *spec,
                                   uint8_t *dst_rgb) {
    int64_t pixels = (int64_t)spec->width * (int64_t)spec->height;
    int64_t i = 0;

    if (spec->channels == 1) {
        double max_value = 0.0;
        for (i = 0; i < pixels; ++i) {
            double value = (double)src[i];
            if (isfinite(value) && value > max_value) {
                max_value = value;
            }
        }
        for (i = 0; i < pixels; ++i) {
            uint8_t v = normalize_float_sample_by_max((double)src[i], max_value);
            dst_rgb[i * 3 + 0] = v;
            dst_rgb[i * 3 + 1] = v;
            dst_rgb[i * 3 + 2] = v;
        }
        return;
    }

    if (spec->layout == IMAGE_LAYOUT_CHW) {
        int64_t plane = pixels;
        for (i = 0; i < pixels; ++i) {
            uint8_t r = normalize_float_sample((double)src[i]);
            uint8_t g = (spec->channels > 1)
                            ? normalize_float_sample((double)src[plane + i])
                            : r;
            uint8_t b = (spec->channels > 2)
                            ? normalize_float_sample((double)src[(plane * 2) + i])
                            : 0u;
            dst_rgb[i * 3 + 0] = r;
            dst_rgb[i * 3 + 1] = g;
            dst_rgb[i * 3 + 2] = b;
        }
        return;
    }

    for (i = 0; i < pixels; ++i) {
        int64_t base = i * (int64_t)spec->channels;
        uint8_t r = normalize_float_sample((double)src[base + 0]);
        uint8_t g = (spec->channels > 1)
                        ? normalize_float_sample((double)src[base + 1])
                        : r;
        uint8_t b = (spec->channels > 2)
                        ? normalize_float_sample((double)src[base + 2])
                        : 0u;
        dst_rgb[i * 3 + 0] = r;
        dst_rgb[i * 3 + 1] = g;
        dst_rgb[i * 3 + 2] = b;
    }
}

static void image_view_convert_f64(const double *src,
                                   const image_spec *spec,
                                   uint8_t *dst_rgb) {
    int64_t pixels = (int64_t)spec->width * (int64_t)spec->height;
    int64_t i = 0;

    if (spec->channels == 1) {
        double max_value = 0.0;
        for (i = 0; i < pixels; ++i) {
            double value = src[i];
            if (isfinite(value) && value > max_value) {
                max_value = value;
            }
        }
        for (i = 0; i < pixels; ++i) {
            uint8_t v = normalize_float_sample_by_max(src[i], max_value);
            dst_rgb[i * 3 + 0] = v;
            dst_rgb[i * 3 + 1] = v;
            dst_rgb[i * 3 + 2] = v;
        }
        return;
    }

    if (spec->layout == IMAGE_LAYOUT_CHW) {
        int64_t plane = pixels;
        for (i = 0; i < pixels; ++i) {
            uint8_t r = normalize_float_sample(src[i]);
            uint8_t g = (spec->channels > 1)
                            ? normalize_float_sample(src[plane + i])
                            : r;
            uint8_t b = (spec->channels > 2)
                            ? normalize_float_sample(src[(plane * 2) + i])
                            : 0u;
            dst_rgb[i * 3 + 0] = r;
            dst_rgb[i * 3 + 1] = g;
            dst_rgb[i * 3 + 2] = b;
        }
        return;
    }

    for (i = 0; i < pixels; ++i) {
        int64_t base = i * (int64_t)spec->channels;
        uint8_t r = normalize_float_sample(src[base + 0]);
        uint8_t g = (spec->channels > 1)
                        ? normalize_float_sample(src[base + 1])
                        : r;
        uint8_t b = (spec->channels > 2)
                        ? normalize_float_sample(src[base + 2])
                        : 0u;
        dst_rgb[i * 3 + 0] = r;
        dst_rgb[i * 3 + 1] = g;
        dst_rgb[i * 3 + 2] = b;
    }
}

#undef IMAGE_VIEW_CONVERT_IMPL

static int image_view_decode_message_rgb(mirage_msg *msg,
                                         const image_view_options *options,
                                         uint8_t *out_rgb,
                                         int max_bytes,
                                         int *out_w,
                                         int *out_h,
                                         int *out_channels) {
    mirage_array_type type = MIRAGE_ARRAY_U8;
    int64_t rank = 0;
    int64_t dims[IMAGE_VIEW_MAX_IMAGE_RANK] = {0, 0, 0};
    image_spec spec;
    int64_t pixels = 0;

    if (!msg || !options || !out_rgb || max_bytes <= 0) {
        return -1;
    }

    if (mirage_read_start(msg) < 0) {
        g_status = IMAGE_VIEW_STATUS_NOT_NUMERIC_ARRAY;
        return -1;
    }
    if (mirage_read_numericarray(msg, &type, &rank, dims, IMAGE_VIEW_MAX_IMAGE_RANK,
                                 NULL, MIRAGE_SKIP_DATA) < 0) {
        g_status = IMAGE_VIEW_STATUS_NOT_NUMERIC_ARRAY;
        return -1;
    }
    if (!is_supported_image_type(type)) {
        g_status = IMAGE_VIEW_STATUS_UNSUPPORTED_TYPE;
        return -1;
    }
    if (!resolve_image_spec(options, type, rank, dims, &spec)) {
        g_status = IMAGE_VIEW_STATUS_BAD_SHAPE;
        return -1;
    }

    pixels = (int64_t)spec.width * (int64_t)spec.height;
    if (pixels <= 0 || pixels > (int64_t)(max_bytes / 3)) {
        g_status = IMAGE_VIEW_STATUS_BUFFER_TOO_SMALL;
        return -1;
    }
    if ((uint64_t)spec.byte_count > g_options.input_buffer_size) {
        g_status = IMAGE_VIEW_STATUS_BUFFER_TOO_SMALL;
        return -1;
    }

    if (mirage_read_start(msg) < 0) {
        g_status = IMAGE_VIEW_STATUS_PAYLOAD_READ_ERROR;
        return -1;
    }
    if (mirage_read_numericarray(msg, &type, &rank, dims, IMAGE_VIEW_MAX_IMAGE_RANK,
                                 g_raw, spec.byte_count) < 0) {
        g_status = IMAGE_VIEW_STATUS_PAYLOAD_READ_ERROR;
        return -1;
    }

    switch (spec.type) {
        case MIRAGE_ARRAY_U8:
            image_view_convert_u8((const uint8_t *)g_raw, &spec, out_rgb);
            break;
        case MIRAGE_ARRAY_U16:
            image_view_convert_u16((const uint16_t *)g_raw, &spec, out_rgb);
            break;
        case MIRAGE_ARRAY_F32:
            image_view_convert_f32((const float *)g_raw, &spec, out_rgb);
            break;
        case MIRAGE_ARRAY_F64:
            image_view_convert_f64((const double *)g_raw, &spec, out_rgb);
            break;
        default:
            g_status = IMAGE_VIEW_STATUS_UNSUPPORTED_TYPE;
            return -1;
    }

    if (out_w) {
        *out_w = spec.width;
    }
    if (out_h) {
        *out_h = spec.height;
    }
    if (out_channels) {
        *out_channels = 3;
    }
    g_status = IMAGE_VIEW_STATUS_FRAME_READY;
    return (int)pixels;
}

static int image_view_bbox_enabled(void) {
    return g_options.bbox_topic[0] != '\0';
}

static void image_view_clear_bboxes(void) {
    g_bbox_count = 0;
}

static int image_view_reserve_bboxes(image_view_bbox **boxes,
                                     int *capacity,
                                     int required) {
    int new_capacity = 0;
    image_view_bbox *resized = NULL;

    if (!boxes || !capacity || required < 0) {
        return 0;
    }
    if (required <= *capacity) {
        return 1;
    }

    new_capacity = *capacity > 0 ? *capacity : IMAGE_VIEW_INITIAL_BOX_CAPACITY;
    while (new_capacity < required) {
        if (new_capacity > INT32_MAX / 2) {
            return 0;
        }
        new_capacity *= 2;
    }

    resized = (image_view_bbox *)realloc(*boxes,
                                         sizeof(**boxes) * (size_t)new_capacity);
    if (!resized) {
        return 0;
    }
    memset(&resized[*capacity],
           0,
           sizeof(*resized) * (size_t)(new_capacity - *capacity));
    *boxes = resized;
    *capacity = new_capacity;
    return 1;
}

static uint8_t image_view_bbox_color(double value) {
    if (!isfinite(value) || value <= 0.0) {
        return 0u;
    }
    if (value <= 1.0) {
        value *= 255.0;
    }
    if (value >= 255.0) {
        return 255u;
    }
    return (uint8_t)lround(value);
}

static int image_view_name_eq(const char *name,
                              int64_t name_len,
                              const char *expected) {
    int64_t expected_len = 0;
    if (!name || !expected || name_len < 0) {
        return 0;
    }
    expected_len = (int64_t)strlen(expected);
    return name_len == expected_len && memcmp(name, expected, (size_t)name_len) == 0;
}

static int image_view_read_number_f64(mirage_msg *msg, double *out_value) {
    mirage_element_type elem_type = MIRAGE_F64;
    int8_t value_i8 = 0;
    int16_t value_i16 = 0;
    int32_t value_i32 = 0;
    int64_t value_i64 = 0;

    if (!msg || !out_value || mirage_peek_element_type(msg, &elem_type) < 0) {
        return 0;
    }

    switch (elem_type) {
        case MIRAGE_F64:
            return mirage_read_f64(msg, out_value) >= 0;
        case MIRAGE_I8:
            if (mirage_read_i8(msg, &value_i8) < 0) {
                return 0;
            }
            *out_value = (double)value_i8;
            return 1;
        case MIRAGE_I16:
            if (mirage_read_i16(msg, &value_i16) < 0) {
                return 0;
            }
            *out_value = (double)value_i16;
            return 1;
        case MIRAGE_I32:
            if (mirage_read_i32(msg, &value_i32) < 0) {
                return 0;
            }
            *out_value = (double)value_i32;
            return 1;
        case MIRAGE_I64:
            if (mirage_read_i64(msg, &value_i64) < 0) {
                return 0;
            }
            *out_value = (double)value_i64;
            return 1;
        default:
            return 0;
    }
}

static int image_view_decode_bbox_values(mirage_msg *msg,
                                         int64_t fn_argc,
                                         image_view_bbox *out_bbox) {
    double color_r = 255.0;
    double color_g = 224.0;
    double color_b = 64.0;
    int64_t name_len = 0;
    image_view_bbox box;

    if (!msg || !out_bbox) {
        return 0;
    }

    if (fn_argc != 4 && fn_argc != 5 && fn_argc != 7 && fn_argc != 8) {
        return 0;
    }

    memset(&box, 0, sizeof(box));
    box.r = 255u;
    box.g = 224u;
    box.b = 64u;

    if (!image_view_read_number_f64(msg, &box.xmin) ||
        !image_view_read_number_f64(msg, &box.ymin) ||
        !image_view_read_number_f64(msg, &box.xmax) ||
        !image_view_read_number_f64(msg, &box.ymax)) {
        return 0;
    }

    if (fn_argc == 5) {
        if (mirage_read_string(msg, box.name, &name_len,
                               (int64_t)sizeof(box.name) - 1) < 0) {
            return 0;
        }
    } else if (fn_argc >= 7) {
        if (!image_view_read_number_f64(msg, &color_r) ||
            !image_view_read_number_f64(msg, &color_g) ||
            !image_view_read_number_f64(msg, &color_b)) {
            return 0;
        }
        box.r = image_view_bbox_color(color_r);
        box.g = image_view_bbox_color(color_g);
        box.b = image_view_bbox_color(color_b);
        if (fn_argc == 8 &&
            mirage_read_string(msg, box.name, &name_len,
                               (int64_t)sizeof(box.name) - 1) < 0) {
            return 0;
        }
    }

    if (!isfinite(box.xmin) || !isfinite(box.ymin) ||
        !isfinite(box.xmax) || !isfinite(box.ymax)) {
        return 0;
    }

    if (name_len < 0) {
        name_len = 0;
    }
    if (name_len >= (int64_t)sizeof(box.name)) {
        name_len = (int64_t)sizeof(box.name) - 1;
    }
    box.name[name_len] = '\0';
    box.name_len = (int)name_len;
    box.valid = 1;
    *out_bbox = box;
    return 1;
}

static int image_view_decode_bbox_list(mirage_msg *msg, image_view_bbox *out_bbox) {
    char fn_name[32];
    int64_t fn_len = 0;
    int64_t fn_argc = 0;

    if (!msg || !out_bbox) {
        return 0;
    }
    if (mirage_read_fn(msg, fn_name, &fn_len, (int64_t)sizeof(fn_name) - 1, &fn_argc) < 0 ||
        !image_view_name_eq(fn_name, fn_len, "List")) {
        return 0;
    }
    return image_view_decode_bbox_values(msg, fn_argc, out_bbox);
}

static int image_view_count_bbox_message(mirage_msg *msg, int *out_count) {
    char fn_name[32];
    int64_t fn_len = 0;
    int64_t fn_argc = 0;
    mirage_element_type elem_type = MIRAGE_FUNCTION;
    static const uint8_t kEmptyList[] = {'8', ':', 'f', 0, 's', 4, 'L', 'i', 's', 't'};

    if (!msg || !out_count) {
        return 0;
    }
    *out_count = 0;
    if (msg->length == (int64_t)sizeof(kEmptyList) &&
        memcmp(msg->data, kEmptyList, sizeof(kEmptyList)) == 0) {
        return 1;
    }
    if (mirage_read_start(msg) < 0 ||
        mirage_read_fn(msg, fn_name, &fn_len, (int64_t)sizeof(fn_name) - 1, &fn_argc) < 0 ||
        !image_view_name_eq(fn_name, fn_len, "List") ||
        fn_argc < 0 ||
        fn_argc > INT32_MAX) {
        return 0;
    }
    if (fn_argc == 0) {
        return 1;
    }
    if (mirage_peek_element_type(msg, &elem_type) < 0) {
        return 0;
    }

    *out_count = elem_type == MIRAGE_FUNCTION ? (int)fn_argc : 1;
    return 1;
}

static int image_view_decode_bboxes(mirage_msg *msg,
                                    image_view_bbox *out_boxes,
                                    int max_boxes,
                                    int *out_count) {
    char fn_name[32];
    int64_t fn_len = 0;
    int64_t fn_argc = 0;
    mirage_element_type elem_type = MIRAGE_FUNCTION;
    int count = 0;
    int64_t i = 0;

    if (!msg || !out_boxes || max_boxes < 0 || !out_count) {
        return 0;
    }
    *out_count = 0;
    if (mirage_read_start(msg) < 0 ||
        mirage_read_fn(msg, fn_name, &fn_len, (int64_t)sizeof(fn_name) - 1, &fn_argc) < 0 ||
        !image_view_name_eq(fn_name, fn_len, "List") ||
        fn_argc < 0) {
        return 0;
    }
    if (fn_argc == 0) {
        return 1;
    }

    if (mirage_peek_element_type(msg, &elem_type) < 0) {
        return 0;
    }

    if (elem_type != MIRAGE_FUNCTION) {
        if (max_boxes < 1) {
            return 1;
        }
        if (!image_view_decode_bbox_values(msg, fn_argc, &out_boxes[0])) {
            return 0;
        }
        *out_count = 1;
        return 1;
    }

    for (i = 0; i < fn_argc && count < max_boxes; ++i) {
        if (!image_view_decode_bbox_list(msg, &out_boxes[count])) {
            return 0;
        }
        ++count;
    }
    *out_count = count;
    return 1;
}

static void image_view_sleep_ns(uint64_t ns) {
    if (ns > 0) {
        rose_nanosleep((double)ns);
    }
}

static int image_view_try_connect(uint64_t now_ns) {
    if (g_sub) {
        return 1;
    }
    if (now_ns - g_last_retry_ns < IMAGE_VIEW_CONNECT_RETRY_NS) {
        g_status = IMAGE_VIEW_STATUS_WAITING_FOR_SUBSCRIBER;
        return 0;
    }
    g_last_retry_ns = now_ns;
    g_sub = rose_create_sub(g_node,
                            g_options.input_topic,
                            -1,
                            (uint64_t)(g_options.latest_only ? 1u : 0u),
                            NULL);
    if (!g_sub) {
        g_status = IMAGE_VIEW_STATUS_WAITING_FOR_SUBSCRIBER;
        return 0;
    }
    return 1;
}

static int image_view_try_connect_bbox(uint64_t now_ns) {
    if (!image_view_bbox_enabled()) {
        return 0;
    }
    if (g_bbox_sub) {
        return 1;
    }
    if (now_ns - g_bbox_last_retry_ns < IMAGE_VIEW_CONNECT_RETRY_NS) {
        return 0;
    }
    g_bbox_last_retry_ns = now_ns;
    g_bbox_sub = rose_create_sub(g_node,
                                 g_options.bbox_topic,
                                 -1,
                                 1u,
                                 NULL);
    return g_bbox_sub != NULL;
}

static void image_view_poll_bbox(uint64_t now_ns) {
    int64_t read_rc = 0;
    image_view_bbox *swap_boxes = NULL;
    int swap_capacity = 0;
    int expected_count = 0;
    int box_count = 0;

    if (!image_view_bbox_enabled() || !g_node || !g_bbox_msg) {
        return;
    }
    if (!image_view_try_connect_bbox(now_ns)) {
        return;
    }

    g_bbox_msg->length = g_bbox_msg->max_length;
    read_rc = rose_readlast(g_bbox_sub, g_bbox_msg);
    if (read_rc >= 0) {
        g_bbox_msg->cursor = 0;
        if (!image_view_count_bbox_message(g_bbox_msg, &expected_count)) {
            return;
        }
        if (expected_count == 0) {
            image_view_clear_bboxes();
            return;
        }
        if (!image_view_reserve_bboxes(&g_bbox_scratch,
                                       &g_bbox_scratch_capacity,
                                       expected_count)) {
            return;
        }
        if (!image_view_decode_bboxes(g_bbox_msg,
                                      g_bbox_scratch,
                                      g_bbox_scratch_capacity,
                                      &box_count)) {
            return;
        }

        swap_boxes = g_bboxes;
        swap_capacity = g_bbox_capacity;
        g_bboxes = g_bbox_scratch;
        g_bbox_capacity = g_bbox_scratch_capacity;
        g_bbox_scratch = swap_boxes;
        g_bbox_scratch_capacity = swap_capacity;
        g_bbox_count = box_count;
        return;
    }
}

static int image_view_wait_for_initial_connection(void) {
    fprintf(stderr,
            "image_view: waiting for topic connection: %s\n",
            g_options.input_topic);

    while (g_node && rose_ok(g_node)) {
        rose_spin_once(g_node);
        if (image_view_try_connect(rose_now())) {
            g_status = IMAGE_VIEW_STATUS_WAITING_FOR_FRAME;
            fprintf(stderr,
                    "image_view: connected topic=%s\n",
                    g_options.input_topic);
            return 1;
        }
        image_view_sleep_ns(IMAGE_VIEW_IDLE_SLEEP_NS);
    }
    return 0;
}

int32_t image_view_bridge_init(uint8_t *topic,
                               uint8_t *bbox_topic,
                               int32_t width,
                               int32_t height,
                               int32_t channels,
                               int32_t input_buffer_size,
                               int32_t latest_only) {
    char node_name[] = "image_view";
    char *argv[] = {node_name, NULL};
    if (!topic || topic[0] == 0 || !bbox_topic || width < 0 || height < 0 ||
        channels < 0 || input_buffer_size <= 0 ||
        (latest_only != 0 && latest_only != 1)) {
        fprintf(stderr, "image_view: invalid init parameters\n");
        return -1;
    }

    memset(&g_options, 0, sizeof(g_options));
    snprintf(g_options.input_topic, sizeof(g_options.input_topic), "%s", (char *)topic);
    snprintf(g_options.bbox_topic, sizeof(g_options.bbox_topic), "%s", (char *)bbox_topic);
    g_options.width = width;
    g_options.height = height;
    g_options.channels = channels;
    g_options.input_buffer_size = (uint64_t)input_buffer_size;
    g_options.latest_only = latest_only;

    g_node = rose_init(1, argv, node_name, NULL, NULL);
    if (!g_node) {
        fprintf(stderr, "image_view: failed to initialize ros3 node: %s\n", rose_last_error());
        return -1;
    }

    g_msg = mirage_create((int64_t)g_options.input_buffer_size, NULL);
    if (!g_msg) {
        fprintf(stderr, "image_view: failed to allocate input buffer\n");
        rose_shutdown(g_node);
        g_node = NULL;
        return -1;
    }

    g_raw = (uint8_t *)malloc((size_t)g_options.input_buffer_size);
    if (!g_raw) {
        fprintf(stderr, "image_view: failed to allocate decode scratch buffer\n");
        mirage_destroy(&g_msg, NULL);
        rose_shutdown(g_node);
        g_node = NULL;
        return -1;
    }

    if (image_view_bbox_enabled()) {
        g_bbox_msg = mirage_create(IMAGE_VIEW_BBOX_BUFFER_BYTES, NULL);
        if (!g_bbox_msg) {
            fprintf(stderr, "image_view: failed to allocate bbox buffer\n");
            free(g_raw);
            g_raw = NULL;
            mirage_destroy(&g_msg, NULL);
            rose_shutdown(g_node);
            g_node = NULL;
            return -1;
        }
    }

    g_sub = NULL;
    g_bbox_sub = NULL;
    g_seq = 0;
    g_last_retry_ns = 0;
    g_bbox_last_retry_ns = 0;
    g_payload_size = 0;
    g_status = IMAGE_VIEW_STATUS_WAITING_FOR_SUBSCRIBER;
    g_bbox_count = 0;

    fprintf(stderr,
            "image_view: topic=%s bbox_topic=%s fallback=%dx%dx%d latest_only=%d buffer=%llu\n",
            g_options.input_topic,
            image_view_bbox_enabled() ? g_options.bbox_topic : "(none)",
            g_options.width,
            g_options.height,
            g_options.channels,
            g_options.latest_only,
            (unsigned long long)g_options.input_buffer_size);
    if (!image_view_wait_for_initial_connection()) {
        fprintf(stderr, "image_view: failed while waiting for topic connection\n");
        image_view_bridge_shutdown();
        return -1;
    }
    return 1;
}

void image_view_bridge_shutdown(void) {
    if (g_msg) {
        mirage_destroy(&g_msg, NULL);
    }
    if (g_bbox_msg) {
        mirage_destroy(&g_bbox_msg, NULL);
    }
    if (g_raw) {
        free(g_raw);
        g_raw = NULL;
    }
    if (g_bboxes) {
        free(g_bboxes);
        g_bboxes = NULL;
    }
    if (g_bbox_scratch) {
        free(g_bbox_scratch);
        g_bbox_scratch = NULL;
    }
    if (g_node) {
        rose_shutdown(g_node);
        g_node = NULL;
    }
    g_sub = NULL;
    g_bbox_sub = NULL;
    g_seq = 0;
    g_last_retry_ns = 0;
    g_bbox_last_retry_ns = 0;
    g_payload_size = 0;
    g_status = IMAGE_VIEW_STATUS_STARTING;
    g_bbox_capacity = 0;
    g_bbox_scratch_capacity = 0;
    g_bbox_count = 0;
}

void image_view_bridge_poll(void) {
    uint64_t wait_start = 0;
    int64_t read_rc = 0;

    if (!g_node || !rose_ok(g_node) || !g_msg) {
        return;
    }

    wait_start = rose_now();
    while (rose_ok(g_node)) {
        rose_spin_once(g_node);
        image_view_poll_bbox(rose_now());

        if (!image_view_try_connect(rose_now())) {
            image_view_sleep_ns(IMAGE_VIEW_IDLE_SLEEP_NS);
            return;
        }

        g_msg->length = g_msg->max_length;
        read_rc = g_options.latest_only ? rose_readlast(g_sub, g_msg) : rose_read(g_sub, g_msg);
        if (read_rc >= 0) {
            g_payload_size = (int)g_msg->length;
            g_msg->cursor = 0;
            g_seq += 1;
            g_status = IMAGE_VIEW_STATUS_FRAME_READY;
            return;
        }
        if (read_rc < ROSE_NO_DATA) {
            g_status = IMAGE_VIEW_STATUS_READ_ERROR;
            image_view_sleep_ns(IMAGE_VIEW_IDLE_SLEEP_NS);
            return;
        }

        g_status = IMAGE_VIEW_STATUS_WAITING_FOR_FRAME;
        if (rose_now() - wait_start >= IMAGE_VIEW_WAIT_BUDGET_NS) {
            image_view_sleep_ns(IMAGE_VIEW_IDLE_SLEEP_NS);
            return;
        }
        image_view_sleep_ns(IMAGE_VIEW_IDLE_SLEEP_NS);
    }
}

uint64_t image_view_bridge_seq(void) {
    return g_seq;
}

int image_view_bridge_status(void) {
    return g_status;
}

int image_view_bridge_payload_size(void) {
    return g_payload_size;
}

int image_view_bridge_bbox_count(void) {
    if (g_node && rose_ok(g_node) && image_view_bbox_enabled()) {
        rose_spin_once(g_node);
        image_view_poll_bbox(rose_now());
    }
    return g_bbox_count;
}

int image_view_bridge_copy_bbox(int index,
                                double *out_xmin,
                                double *out_ymin,
                                double *out_xmax,
                                double *out_ymax,
                                int *out_r,
                                int *out_g,
                                int *out_b,
                                uint8_t *out_name,
                                int out_name_size) {
    int copy_len = 0;
    const image_view_bbox *box = NULL;

    if (!g_bboxes || index < 0 || index >= g_bbox_count) {
        return 0;
    }
    box = &g_bboxes[index];

    if (!box->valid || !out_xmin || !out_ymin || !out_xmax || !out_ymax ||
        !out_r || !out_g || !out_b || !out_name || out_name_size <= 0) {
        return 0;
    }

    *out_xmin = box->xmin;
    *out_ymin = box->ymin;
    *out_xmax = box->xmax;
    *out_ymax = box->ymax;
    *out_r = (int)box->r;
    *out_g = (int)box->g;
    *out_b = (int)box->b;

    copy_len = box->name_len;
    if (copy_len > out_name_size - 1) {
        copy_len = out_name_size - 1;
    }
    if (copy_len > 0) {
        memcpy(out_name, box->name, (size_t)copy_len);
    }
    out_name[copy_len] = 0u;
    return copy_len + 1;
}

int image_view_bridge_copy_rgb(uint8_t *out_rgb,
                               int max_bytes,
                               int *out_w,
                               int *out_h,
                               int *out_channels) {
    if (!g_msg || g_payload_size <= 0) {
        return -1;
    }
    return image_view_decode_message_rgb(g_msg, &g_options, out_rgb, max_bytes,
                                         out_w, out_h, out_channels);
}
