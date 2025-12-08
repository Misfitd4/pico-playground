#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "bsp/board_api.h"
#include "pico.h"
#include "pico/multicore.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/stdlib.h"

#include "tusb.h"

#include "siddler_audio.h"
#include "sid_engine.h"

#define SIDDLER_SYS_CLOCK_KHZ 250000u
#define VGA_MODE vga_mode_320x240_60
#define DUMP_EVENT_SIZE 4u
#define TEXT_COLS 40
#define TEXT_ROWS 27






#define EVENT_QUEUE_CAP 4096
#define FLOW_HIGH_WATER (EVENT_QUEUE_CAP - 128)
#define FLOW_LOW_WATER 256u
#define RECENT_BUF_SIZE 512u

#define SID_CLOCK_HZ 985248u
#define CLOCK_SCALE_BASE 1000000u
#define CLOCK_SCALE_MIN 200000u   /* 0.20x */
#define CLOCK_SCALE_MAX 3000000u  /* 3.00x */
#define SID_DELAY_ADDR 0xFFu

#ifndef COUNT_OF
#define COUNT_OF(x) (sizeof(x) / sizeof((x)[0]))
#endif
#define SID_ENGINE_QUEUE_HIGH_WATER 6000u

typedef struct {
    uint8_t buf[DUMP_EVENT_SIZE];
    size_t have;
    uint64_t total_events;
    uint64_t total_bytes;
    uint64_t total_cycles;
    uint32_t last_delta;
    uint8_t last_addr;
    uint8_t last_value;
    bool streaming;
} dump_loader_t;

typedef struct {
    uint32_t delta;
    uint8_t addr;
    uint8_t value;
} dump_event_t;

typedef struct {
    dump_event_t events[EVENT_QUEUE_CAP];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} dump_queue_t;

typedef enum {
    VIEW_STATUS = 0,
    VIEW_USB_QUEUE,
    VIEW_SID_QUEUE,
    VIEW_HEX,
    VIEW_COUNT
} debug_view_t;

static dump_loader_t g_loader;
static bool g_paused = false;
static bool g_audio_ready = false;
static bool g_ready_sent = false;
static dump_queue_t g_queue;
static uint32_t g_clock_scale_ppm = CLOCK_SCALE_BASE;
static debug_view_t g_active_view = VIEW_STATUS;
static uint32_t g_queue_max_depth = 0;
static uint64_t g_queue_cycles = 0;
static bool g_flow_paused = false;
static uint8_t g_recent_bytes[RECENT_BUF_SIZE];
static size_t g_recent_head = 0;
static bool g_recent_full = false;
static uint64_t g_recent_total_bytes = 0;

typedef enum {
    SID_MODE_6581 = 0,
    SID_MODE_8580,
    SID_MODE_SPLIT,
    SID_MODE_COUNT
} sid_mode_t;

static sid_mode_t g_sid_mode = SID_MODE_6581;

static void buttons_init(void);
static void process_buttons(void);
static void sid_mode_apply(bool announce);
static void sid_mode_cycle(void);

#if defined(VGABOARD_BUTTON_A_PIN) && defined(VGABOARD_BUTTON_B_PIN) && defined(VGABOARD_BUTTON_C_PIN)
#define SIDDLER_USE_VGABOARD_BUTTONS 1
#else
#define SIDDLER_USE_VGABOARD_BUTTONS 0
#endif

#if SIDDLER_USE_VGABOARD_BUTTONS
static volatile uint32_t g_button_state = 0;
static uint32_t g_button_last = 0;
static const uint g_button_pins[] = {
    VGABOARD_BUTTON_A_PIN,
    VGABOARD_BUTTON_B_PIN,
    VGABOARD_BUTTON_C_PIN,
};
static const uint VSYNC_PIN = PICO_SCANVIDEO_COLOR_PIN_BASE +
                              PICO_SCANVIDEO_COLOR_PIN_COUNT + 1u;
#endif

static critical_section_t g_status_lock;
static char g_status_lines[TEXT_ROWS][TEXT_COLS];

static inline uint16_t *raw_scanline_prepare(struct scanvideo_scanline_buffer *dest, uint width)
{
    assert(width >= 3);
    assert((width & 1u) == 0);
    dest->data[0] = COMPOSABLE_RAW_RUN | ((width + 1 - 3) << 16);
    dest->data[width / 2 + 2] = 0x0000 | (COMPOSABLE_EOL_ALIGN << 16);
    dest->data_used = width / 2 + 2;
    assert(dest->data_used <= dest->data_max);
    return (uint16_t *) &dest->data[1];
}

static inline void raw_scanline_finish(struct scanvideo_scanline_buffer *dest)
{
    uint32_t first = dest->data[0];
    uint32_t second = dest->data[1];
    dest->data[0] = (first & 0x0000ffffu) | ((second & 0x0000ffffu) << 16);
    dest->data[1] = (second & 0xffff0000u) | ((first & 0xffff0000u) >> 16);
    dest->status = SCANLINE_OK;
}

/* ------------------------- Text rendering ------------------------- */

static const uint8_t font8x8_basic[128][8] = {
#include "../font8x8_basic.inc"
};

static void clear_status_lines(void)
{
    critical_section_enter_blocking(&g_status_lock);
    for (int row = 0; row < TEXT_ROWS; ++row) {
        memset(g_status_lines[row], ' ', TEXT_COLS);
    }
    critical_section_exit(&g_status_lock);
}

static void set_status_line(int row, const char *fmt, ...)
{
    if (row < 0 || row >= TEXT_ROWS) return;
    char temp[TEXT_COLS + 1];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(temp, sizeof temp, fmt, args);
    va_end(args);
    if (written < 0) written = 0;
    if (written > TEXT_COLS) written = TEXT_COLS;

    critical_section_enter_blocking(&g_status_lock);
    memset(g_status_lines[row], ' ', TEXT_COLS);
    memcpy(g_status_lines[row], temp, (size_t) written);
    critical_section_exit(&g_status_lock);
}

static void __time_critical_func(render_text_screen_scanline)(
    struct scanvideo_scanline_buffer *dest)
{
    int scanline  = scanvideo_scanline_number(dest->scanline_id);
    int row       = scanline / 8;
    int glyph_row = scanline % 8;
    uint16_t *pixels = raw_scanline_prepare(dest, VGA_MODE.width);
    uint16_t bg = PICO_SCANVIDEO_PIXEL_FROM_RGB8(4, 6, 12);
    for (int x = 0; x < VGA_MODE.width; ++x) pixels[x] = bg;

    if (row >= 0 && row < TEXT_ROWS) {
        char row_chars[TEXT_COLS];
        critical_section_enter_blocking(&g_status_lock);
        memcpy(row_chars, g_status_lines[row], TEXT_COLS);
        critical_section_exit(&g_status_lock);
        for (int col = 0; col < TEXT_COLS; ++col) {
            uint8_t ch = (uint8_t) row_chars[col];
            uint8_t glyph = font8x8_basic[ch & 0x7Fu][glyph_row & 7];
            uint16_t fg = PICO_SCANVIDEO_PIXEL_FROM_RGB8(
                (row == 0) ? 200 : 120,
                (row == 0) ? 220 : 180,
                (row == 0) ? 255 : 200);
            for (int bit = 0; bit < 8; ++bit) {
                int x = col * 8 + bit;
                if (x >= VGA_MODE.width) break;
                if (glyph & (1u << (7 - bit))) {
                    pixels[x] = fg;
                }
            }
        }
    }

    raw_scanline_finish(dest);
}

static void __time_critical_func(render_loop)(void)
{
    while (true) {
        struct scanvideo_scanline_buffer *scanline_buffer =
            scanvideo_begin_scanline_generation(true);
        render_text_screen_scanline(scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}

/* ------------------------- Loader ------------------------- */

static void flow_control_set(bool paused)
{
    if (paused == g_flow_paused) {
        return;
    }
    g_flow_paused = paused;
}

static void flow_control_consider(void)
{
    if (!tud_cdc_connected()) {
        g_flow_paused = false;
        return;
    }
    if (!g_flow_paused && g_queue.count >= FLOW_HIGH_WATER) {
        flow_control_set(true);
    } else if (g_flow_paused && g_queue.count <= FLOW_LOW_WATER) {
        flow_control_set(false);
    }
}

static void dump_queue_reset(dump_queue_t *queue)
{
    queue->head = queue->tail = queue->count = 0;
    g_queue_max_depth = 0;
    g_queue_cycles = 0;
    if (g_flow_paused) {
        flow_control_set(false);
    }
}

static void dump_loader_reset(dump_loader_t *loader,
                              bool preserve_stats,
                              bool reset_audio)
{
    uint64_t total_events = loader->total_events;
    uint64_t total_bytes  = loader->total_bytes;
    uint64_t total_cycles = loader->total_cycles;
    uint32_t last_delta   = loader->last_delta;
    uint8_t  last_addr    = loader->last_addr;
    uint8_t  last_value   = loader->last_value;

    loader->have = 0;
    if (!preserve_stats) {
        total_events = 0;
        total_bytes  = 0;
        total_cycles = 0;
        last_delta   = 0;
        last_addr    = 0;
        last_value   = 0;
    }
    loader->total_events = total_events;
    loader->total_bytes  = total_bytes;
    loader->total_cycles = total_cycles;
    loader->last_delta   = last_delta;
    loader->last_addr    = last_addr;
    loader->last_value   = last_value;
    loader->streaming    = false;

    dump_queue_reset(&g_queue);

    if (reset_audio) {
        siddler_audio_reset_state();
        if (g_audio_ready) {
            sid_mode_apply(false);
        }
    }
}

static void recent_bytes_record(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        g_recent_bytes[g_recent_head] = data[i];
        g_recent_head = (g_recent_head + 1u) % RECENT_BUF_SIZE;
        if (g_recent_head == 0) {
            g_recent_full = true;
        }
    }
    g_recent_total_bytes += len;
}

static bool dump_queue_push(dump_queue_t *queue,
                            uint32_t delta, uint8_t addr, uint8_t value)
{
    // If full, drop the oldest event (advance head) to make room.
    if (queue->count >= EVENT_QUEUE_CAP) {
        dump_event_t *old = &queue->events[queue->head];

        // Keep g_queue_cycles consistent
        if (g_queue_cycles >= old->delta) {
            g_queue_cycles -= old->delta;
        } else {
            g_queue_cycles = 0;
        }

        queue->head = (queue->head + 1u) % EVENT_QUEUE_CAP;
        queue->count--;
    }

    dump_event_t *ev = &queue->events[queue->tail];
    ev->delta = delta;
    ev->addr  = addr;
    ev->value = value;

    queue->tail = (queue->tail + 1u) % EVENT_QUEUE_CAP;
    queue->count++;
    g_queue_cycles += delta;

    return true; // push never "fails" now
}

static bool dump_queue_push1(dump_queue_t *queue,
                             uint32_t delta, uint8_t addr, uint8_t value)
{
    if (queue->count >= EVENT_QUEUE_CAP) {
        return false;
    }
    dump_event_t *ev = &queue->events[queue->tail];
    ev->delta = delta;
    ev->addr  = addr;
    ev->value = value;
    queue->tail = (queue->tail + 1u) % EVENT_QUEUE_CAP;
    queue->count++;
    g_queue_cycles += delta;
    return true;
}

static bool dump_queue_pop(dump_queue_t *queue, dump_event_t *out)
{
    if (queue->count == 0) {
        return false;
    }
    dump_event_t *ev = &queue->events[queue->head];
    if (out) {
        *out = *ev;
    }
    if (g_queue_cycles >= ev->delta) {
        g_queue_cycles -= ev->delta;
    } else {
        g_queue_cycles = 0;
    }
    queue->head = (queue->head + 1u) % EVENT_QUEUE_CAP;
    queue->count--;
    return true;
}

static void dump_loader_handle_event(dump_loader_t *loader,
                                     uint32_t delta, uint8_t addr, uint8_t value)
{
    loader->total_events++;
    loader->total_cycles += delta;
    loader->last_delta = delta;
    loader->last_addr  = addr;
    loader->last_value = value;

    // This never fails now – on full it drops the oldest.
    dump_queue_push(&g_queue, delta, addr, value);

    if (g_queue.count > g_queue_max_depth) {
        g_queue_max_depth = g_queue.count;
    }
    flow_control_consider();
}

static void dump_loader_handle_event1(dump_loader_t *loader,
                                      uint32_t delta, uint8_t addr, uint8_t value)
{
    loader->total_events++;
    loader->total_cycles += delta;
    loader->last_delta = delta;
    loader->last_addr  = addr;
    loader->last_value = value;
    if (!dump_queue_push(&g_queue, delta, addr, value)) {
        printf("[DUMP] queue overflow, resetting\n");
        dump_queue_reset(&g_queue);
    }
    if (g_queue.count > g_queue_max_depth) {
        g_queue_max_depth = g_queue.count;
    }
    flow_control_consider();
}

static void dump_loader_feed(dump_loader_t *loader,
                             const uint8_t *data, size_t len)
{
    if (data && len) {
        recent_bytes_record(data, len);
    }
    size_t offset = 0;
    while (offset < len) {
        size_t take = DUMP_EVENT_SIZE - loader->have;
        if (take > len - offset) take = len - offset;
        memcpy(loader->buf + loader->have, data + offset, take);
        loader->have += take;
        offset += take;
        if (loader->have == DUMP_EVENT_SIZE) {
            uint32_t delta = (uint32_t) loader->buf[0] |
                             ((uint32_t) loader->buf[1] << 8);
            uint8_t addr  = loader->buf[2];
            uint8_t value = loader->buf[3];
            dump_loader_handle_event(loader, delta, addr, value);
            loader->have      = 0;
            loader->streaming = true;
        }
    }
}

static uint32_t dump_queue_snapshot(dump_event_t *out, uint32_t max_events)
{
    if (!out || !max_events) return 0;
    uint32_t count = g_queue.count;
    if (count > max_events) count = max_events;
    uint32_t idx = g_queue.head;
    for (uint32_t i = 0; i < count; ++i) {
        out[i] = g_queue.events[idx];
        idx = (idx + 1u) % EVENT_QUEUE_CAP;
    }
    return count;
}

static const char *view_name(debug_view_t view)
{
    static const char *names[VIEW_COUNT] = {
        "STATUS",
        "USB QUEUE",
        "SID QUEUE",
        "HEX DUMP",
    };
    if (view >= VIEW_COUNT) return "UNKNOWN";
    return names[view];
}

static void set_view(debug_view_t view)
{
    if (view >= VIEW_COUNT) {
        view = VIEW_STATUS;
    }
    if (g_active_view == view) {
        return;
    }
    g_active_view = view;
    printf("[DUMP] view -> %s\n", view_name(view));
}

static void cycle_view(int delta)
{
    int next = (int) g_active_view + delta;
    while (next < 0) {
        next += VIEW_COUNT;
    }
    next %= VIEW_COUNT;
    set_view((debug_view_t) next);
}

/* ------------------------- UI / CDC ------------------------- */

static uint64_t effective_clock_hz(void)
{
    return (((uint64_t) SID_CLOCK_HZ * g_clock_scale_ppm) + CLOCK_SCALE_BASE / 2u) /
           CLOCK_SCALE_BASE;
}

static void log_clock_scale(void)
{
    uint32_t percent_whole = g_clock_scale_ppm / 10000u;
    uint32_t percent_frac  = (g_clock_scale_ppm / 100u) % 100u;
    uint64_t hz = effective_clock_hz();
    printf("[DUMP] clock %3u.%02u%% (%lu Hz)\n",
           (unsigned int) percent_whole,
           (unsigned int) percent_frac,
           (unsigned long) hz);
}

static void clock_scale_apply(uint32_t new_ppm)
{
    if (new_ppm < CLOCK_SCALE_MIN) new_ppm = CLOCK_SCALE_MIN;
    if (new_ppm > CLOCK_SCALE_MAX) new_ppm = CLOCK_SCALE_MAX;
    if (new_ppm == g_clock_scale_ppm) {
        return;
    }
    g_clock_scale_ppm = new_ppm;
    log_clock_scale();
}

static void clock_scale_mul(uint32_t num, uint32_t denom)
{
    if (!denom) return;
    uint64_t scaled = ((uint64_t) g_clock_scale_ppm * num + denom / 2u) / denom;
    clock_scale_apply((uint32_t) scaled);
}

static void clock_scale_reset(void)
{
    if (g_clock_scale_ppm == CLOCK_SCALE_BASE) {
        return;
    }
    g_clock_scale_ppm = CLOCK_SCALE_BASE;
    log_clock_scale();
}

static inline uint8_t sid_mode_chip_mask(void)
{
    return (g_sid_mode == SID_MODE_SPLIT) ? 0x3u : 0x1u;
}

static const char *sid_mode_name(sid_mode_t mode)
{
    switch (mode) {
    case SID_MODE_6581:  return "6581";
    case SID_MODE_8580:  return "8580";
    case SID_MODE_SPLIT: return "6581+8580";
    default:             return "?";
    }
}

static void sid_mode_apply(bool announce)
{
    bool left_6581  = (g_sid_mode != SID_MODE_8580);
    bool right_6581 = (g_sid_mode == SID_MODE_SPLIT) ? false : left_6581;
    if (g_audio_ready) {
        sid_engine_set_channel_models(left_6581, right_6581);
        sid_engine_set_split_channels(g_sid_mode == SID_MODE_SPLIT);
    }
    if (announce) {
        printf("[SID] mode %s\n", sid_mode_name(g_sid_mode));
    }
}

static void sid_mode_cycle(void)
{
    g_sid_mode = (sid_mode_t) ((g_sid_mode + 1) % SID_MODE_COUNT);
    sid_mode_apply(true);
}

static inline uint32_t scale_delta_cycles(uint32_t delta)
{
    if (!delta) {
        return 0;
    }
    uint64_t scaled = ((uint64_t) delta * g_clock_scale_ppm +
                       CLOCK_SCALE_BASE / 2u) / CLOCK_SCALE_BASE;
    if (scaled == 0) {
        scaled = 1;
    }
    if (scaled > UINT32_MAX) {
        scaled = UINT32_MAX;
    }
    return (uint32_t) scaled;
}

static void send_ready(void)
{
    if (g_ready_sent) return;
    const char *msg = "[DUMP] READY\r\n";
    tud_cdc_write(msg, strlen(msg));
    tud_cdc_write_flush();
    printf("[DUMP] READY\n");
    g_ready_sent = true;
}

static void render_status_view(void)
{
    uint64_t seconds = g_loader.total_cycles / SID_CLOCK_HZ;
    uint64_t millis  = (g_loader.total_cycles % SID_CLOCK_HZ) * 1000u / SID_CLOCK_HZ;
    sid_engine_queue_stats_t stats;
    sid_engine_get_queue_stats(&stats);

    set_status_line(0, "SIDDLER PICO [%s]", view_name(g_active_view));
    set_status_line(1, "Events:%10llu  Bytes:%10llu",
                    (unsigned long long) g_loader.total_events,
                    (unsigned long long) g_loader.total_bytes);
    set_status_line(2, "Time  : %5llu.%03llus",
                    (unsigned long long) seconds,
                    (unsigned long long) millis);
    set_status_line(3, "Last  : d=%-6lu addr=$%02X val=$%02X",
                    (unsigned long) g_loader.last_delta,
                    g_loader.last_addr, g_loader.last_value);
    set_status_line(4, "Stream:%s  Paused:%s  Audio:%s",
                    g_loader.streaming ? "ON " : "OFF",
                    g_paused ? "YES" : " NO",
                    g_audio_ready ? "OK" : "ERR");
    set_status_line(5, "Clock : %3u.%02u%%  %7lu Hz",
                    (unsigned int) (g_clock_scale_ppm / 10000u),
                    (unsigned int) ((g_clock_scale_ppm / 100u) % 100u),
                    (unsigned long) effective_clock_hz());
    set_status_line(6, "USBQ  : %4u (max %4u) cyc=%8llu %s",
                    g_queue.count, g_queue_max_depth,
                    (unsigned long long) g_queue_cycles,
                    g_flow_paused ? "HALT" : "OK  ");
    set_status_line(7, "SIDQ  : depth=%4u drop=%4u next=%6u",
                    stats.depth, stats.dropped, stats.cycles_to_next);
    set_status_line(8, "View  : %s", view_name(g_active_view));
    set_status_line(9, "SID   : %s", sid_mode_name(g_sid_mode));
}

static void render_usb_queue_view(void)
{
    set_status_line(0, "USB QUEUE depth=%u flow=%s",
                    g_queue.count, g_flow_paused ? "HALT" : "OK");
    set_status_line(1, "Max depth: %u", g_queue_max_depth);
    dump_event_t snapshot[10];
    uint32_t shown = dump_queue_snapshot(snapshot, COUNT_OF(snapshot));
    if (!shown) {
        set_status_line(3, "Queue empty");
        return;
    }
    for (uint32_t i = 0; i < shown && (i + 2) < TEXT_ROWS; ++i) {
        set_status_line((int) (2 + i), "%2u: +%6u addr $%02X = $%02X",
                        i,
                        snapshot[i].delta,
                        snapshot[i].addr,
                        snapshot[i].value);
    }
}

static void render_sid_queue_view(void)
{
    sid_engine_queue_entry_t entries[12];
    uint32_t cycles_to_next = 0;
    size_t got = sid_engine_peek_queue(entries, COUNT_OF(entries), &cycles_to_next);
    sid_engine_queue_stats_t stats;
    sid_engine_get_queue_stats(&stats);

    set_status_line(0, "SID ENGINE QUEUE depth=%u drop=%u",
                    stats.depth, stats.dropped);
    set_status_line(1, "Next event in %u cycles", cycles_to_next);

    if (!got) {
        set_status_line(3, "No pending SID events");
        return;
    }

    for (size_t i = 0; i < got && (i + 2) < TEXT_ROWS; ++i) {
        set_status_line((int) (2 + i), "%2u: +%6u chip %u addr $%02X = $%02X",
                        (unsigned int) i,
                        entries[i].delta,
                        entries[i].chip_mask,
                        entries[i].addr,
                        entries[i].value);
    }
}

static void render_hex_view(void)
{
    size_t available = g_recent_full ? RECENT_BUF_SIZE : g_recent_head;
    set_status_line(0, "HEX RX (total %llu bytes)",
                    (unsigned long long) g_recent_total_bytes);
    if (!available) {
        set_status_line(2, "No data captured yet");
        return;
    }
    const size_t bytes_per_line = 8;
    const size_t max_lines = TEXT_ROWS - 2;
    size_t max_bytes = bytes_per_line * max_lines;
    size_t bytes_to_show = available;
    if (bytes_to_show > max_bytes) {
        bytes_to_show = max_bytes;
    }
    size_t start = (g_recent_head + RECENT_BUF_SIZE - bytes_to_show) % RECENT_BUF_SIZE;
    uint64_t base_index = g_recent_total_bytes - bytes_to_show;

    for (size_t line = 0; line < max_lines; ++line) {
        size_t offset = line * bytes_per_line;
        if (offset >= bytes_to_show) break;
        size_t chunk = bytes_per_line;
        if (offset + chunk > bytes_to_show) {
            chunk = bytes_to_show - offset;
        }
        char buf[TEXT_COLS + 1];
        int len = snprintf(buf, sizeof buf, "%08llX:",
                           (unsigned long long) (base_index + offset));
        for (size_t i = 0; i < chunk && len < (int) sizeof buf; ++i) {
            size_t idx = (start + offset + i) % RECENT_BUF_SIZE;
            len += snprintf(buf + len, sizeof buf - len, " %02X",
                            g_recent_bytes[idx]);
        }
        set_status_line((int) (2 + line), "%s", buf);
    }
}

static void update_status_screen(void)
{
    clear_status_lines();
    switch (g_active_view) {
    case VIEW_STATUS:
        render_status_view();
        break;
    case VIEW_USB_QUEUE:
        render_usb_queue_view();
        break;
    case VIEW_SID_QUEUE:
        render_sid_queue_view();
        break;
    case VIEW_HEX:
        render_hex_view();
        break;
    default:
        render_status_view();
        break;
    }
}

static void process_console(void)
{
    int ch = getchar_timeout_us(0);
    if (ch < 0) return;
    switch (ch) {
    case 'p':
    case 'P':
    case ' ':
        g_paused = !g_paused;
        printf("[DUMP] %s\n", g_paused ? "paused" : "playing");
        break;
    case '1':
        set_view(VIEW_STATUS);
        break;
    case '2':
        set_view(VIEW_USB_QUEUE);
        break;
    case '3':
        set_view(VIEW_SID_QUEUE);
        break;
    case '4':
    case 'h':
    case 'H':
        set_view(VIEW_HEX);
        break;
    case 'v':
    case 'V':
        cycle_view(1);
        break;
    case 'm':
    case 'M':
        sid_mode_cycle();
        break;
    case '[':
        clock_scale_mul(99u, 100u);
        break;
    case ']':
        clock_scale_mul(101u, 100u);
        break;
    case '{':
    case '<':
        clock_scale_mul(9u, 10u);
        break;
    case '}':
    case '>':
        clock_scale_mul(11u, 10u);
        break;
    case '0':
    case '=':
        clock_scale_reset();
        break;
    default:
        break;
    }
}

/* BOUNDED USB READ: avoid hogging CPU when a lot of data is queued */
static void process_serial(void)
{
    // If flow is paused, don't pull more from USB.
    if (g_flow_paused) {
        return;
    }

    const int MAX_CHUNKS = 4; // at most 4 * 512 bytes per call
    int chunks = 0;

    while (tud_cdc_available() && !g_flow_paused && chunks < MAX_CHUNKS) {
        uint8_t buf[512];
        uint32_t count = tud_cdc_read(buf, sizeof buf);
        if (!count) {
            break;
        }
        g_loader.total_bytes += count;
        dump_loader_feed(&g_loader, buf, count);
        chunks++;
        // dump_loader_feed() may change queue depth and thus flow state
        // via flow_control_consider(), so loop condition checks g_flow_paused.
    }
}

static void process_serial1(void)
{
    while (tud_cdc_available()) {
        uint8_t buf[512];
        uint32_t count = tud_cdc_read(buf, sizeof buf);
        if (!count) break;
        g_loader.total_bytes += count;
        dump_loader_feed(&g_loader, buf, count);
    }
}

static void dump_queue_service(void)
{
    if (!g_audio_ready || g_paused) {
        return;
    }
    while (g_queue.count) {
        uint32_t depth = sid_engine_get_queue_depth();
        if (depth > SID_ENGINE_QUEUE_HIGH_WATER) {
            break;
        }
        dump_event_t ev;
        if (!dump_queue_pop(&g_queue, &ev)) {
            break;
        }
        uint32_t scaled_delta = scale_delta_cycles(ev.delta);
        uint8_t chip_mask = sid_mode_chip_mask();
        if (ev.addr != SID_DELAY_ADDR) {
            siddler_audio_queue_event(chip_mask, ev.addr & 0x1F, ev.value, scaled_delta);
        } else {
            siddler_audio_queue_event(0, 0, 0, scaled_delta);
        }
        flow_control_consider();
    }
}

/* ------------------------- Main ------------------------- */

int main(void)
{
    board_init();
#if PICO_SCANVIDEO_48MHZ
    set_sys_clock_48mhz();
#else
    set_sys_clock_khz(SIDDLER_SYS_CLOCK_KHZ, true);
#endif
    setup_default_uart();
    tusb_init();
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    critical_section_init(&g_status_lock);
    clear_status_lines();

    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);
    multicore_launch_core1(render_loop);
    buttons_init();

    // Initial reset: clean loader + SID audio (for first session)
    dump_loader_reset(&g_loader, false, true);
    g_audio_ready = siddler_audio_init();
    printf("siddler_pico dump | audio %s\n", g_audio_ready ? "OK" : "FAIL");
    if (g_audio_ready) {
        sid_mode_apply(true);
    }

    absolute_time_t next_status = make_timeout_time_ms(0);

    while (true) {
        /* ---- 1) AUDIO FIRST: give reSID maximum priority ---- */
        if (g_audio_ready) {
            siddler_audio_task();
        }

        /* ---- 2) Feed SID engine queue from USB event queue ---- */
        dump_queue_service();

        /* ---- 3) USB device maintenance & host connection ---- */
        tud_task();

        if (!tud_cdc_connected()) {
            g_ready_sent = false;
            if (g_loader.have || g_loader.streaming || g_queue.count) {
                // USB gone: clear loader + USB queue, but KEEP SID state
                dump_loader_reset(&g_loader, true, false);
            }
        } else if (!g_ready_sent) {
            // New connection/session: reset loader + SID audio
            dump_loader_reset(&g_loader, false, true);
            send_ready();
        }

        /* ---- 4) Pull some data from host, with bounded work ---- */
        process_serial();

        /* ---- 5) UI / input (cheap when idle) ---- */
        process_buttons();
        process_console();

        /* ---- 6) Update on-screen status at 10 Hz ---- */
        if (absolute_time_diff_us(get_absolute_time(), next_status) <= 0) {
            update_status_screen();
            next_status = delayed_by_us(next_status, 100000);
        }
    }
}

/* ------------------------- Buttons ------------------------- */

#if SIDDLER_USE_VGABOARD_BUTTONS
static void __isr vga_board_button_irq_handler(void)
{
    int vsync_level = gpio_get(VSYNC_PIN);
    gpio_acknowledge_irq(VSYNC_PIN,
                         vsync_level ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL);

    if (vsync_level != scanvideo_get_mode().default_timing->v_sync_polarity) {
        for (size_t i = 0; i < COUNT_OF(g_button_pins); ++i) {
            gpio_pull_down(g_button_pins[i]);
            gpio_set_oeover(g_button_pins[i], GPIO_OVERRIDE_LOW);
        }
    } else {
        uint32_t state = 0;
        for (size_t i = 0; i < COUNT_OF(g_button_pins); ++i) {
            state |= ((uint32_t) gpio_get(g_button_pins[i])) << i;
            gpio_set_oeover(g_button_pins[i], GPIO_OVERRIDE_NORMAL);
        }
        g_button_state = state;
    }
}

static void buttons_init(void)
{
    gpio_set_irq_enabled(VSYNC_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
    irq_set_exclusive_handler(IO_IRQ_BANK0, vga_board_button_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(VSYNC_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
}

static void handle_button_release(uint button_index)
{
    switch (button_index) {
    case 0:
        cycle_view(-1);
        break;
    case 1:
        cycle_view(1);
        break;
    case 2:
        sid_mode_cycle();
        break;
    default:
        break;
    }
}

static void process_buttons(void)
{
    uint32_t state   = g_button_state;
    uint32_t changed = state ^ g_button_last;
    uint32_t released = changed & ~state;
    if (released) {
        for (size_t i = 0; i < COUNT_OF(g_button_pins); ++i) {
            if (released & (1u << i)) {
                handle_button_release((uint) i);
            }
        }
    }
    g_button_last = state;
}
#else
static void buttons_init(void) {}
static void process_buttons(void) {}
#endif
