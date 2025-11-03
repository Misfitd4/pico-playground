/*
 * siddler_pico - Scanvideo visualiser driven by SIDTap serial frames
 *
 * Consumes SID register events forwarded by tools/sidtap2serial over the USB
 * CDC connection and maps activity to a colourful raster.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <ctype.h>

#include "hardware/clocks.h"

#include "bsp/board_api.h"
#include "pico.h"
#include "pico/multicore.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/stdlib.h"
#include "pico/sync.h"

#include "tusb.h"

#include "siddler_audio.h"

#define SIDDLER_PIXEL_CLOCK_HZ 24000000u
#define SIDDLER_SYS_CLOCK_KHZ 240000u

static const scanvideo_timing_t siddler_vga_timing_640x480_50 = {
	.clock_freq = SIDDLER_PIXEL_CLOCK_HZ,
	.h_active = 640,
	.v_active = 480,
	.h_front_porch = 32,
	.h_pulse = 64,
	.h_total = 768,
	.h_sync_polarity = 1,
	.v_front_porch = 10,
	.v_pulse = 2,
	.v_total = 625,
	.v_sync_polarity = 1,
	.enable_clock = 0,
	.clock_polarity = 0,
	.enable_den = 0
};

static const scanvideo_mode_t siddler_vga_mode_320x240_50 = {
	.default_timing = &siddler_vga_timing_640x480_50,
	.pio_program = &video_24mhz_composable,
	.width = 320,
	.height = 240,
	.xscale = 2,
	.yscale = 2,
};

#define VGA_MODE siddler_vga_mode_320x240_50
#define SERIAL_BUFFER_SIZE 4096
#define SID_MAGIC 0x53494446u
#define MIN_COLOR_RUN 3
#define CHAR_WIDTH 8
#define CHAR_HEIGHT 8
#define TEXT_COLS 40
#define TEXT_ROWS 27
#define LOG_ROW_START 6
#define LOG_ROWS (TEXT_ROWS - LOG_ROW_START)
#define EVENT_ROW_START (LOG_ROW_START + 10)
#define EVENT_LOG_MAX 30

/* 8x8 bitmap font (public domain, derived from font8x8_basic by ProjektITU) */
static const uint8_t font8x8_basic[128][8] = {
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x7e,0x81,0xa5,0x81,0xbd,0x99,0x81,0x7e},
        {0x7e,0xff,0xdb,0xff,0xc3,0xe7,0xff,0x7e},
        {0x6c,0xfe,0xfe,0xfe,0x7c,0x38,0x10,0x00},
        {0x10,0x38,0x7c,0xfe,0x7c,0x38,0x10,0x00},
        {0x38,0x7c,0x38,0xfe,0xfe,0xd6,0x10,0x38},
        {0x10,0x38,0x7c,0xfe,0xfe,0x7c,0x10,0x38},
        {0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00},
        {0xff,0xff,0xe7,0xc3,0xc3,0xe7,0xff,0xff},
        {0x00,0x3c,0x66,0x42,0x42,0x66,0x3c,0x00},
        {0xff,0xc3,0x99,0xbd,0xbd,0x99,0xc3,0xff},
        {0x0f,0x07,0x0f,0x7d,0xcc,0xcc,0xcc,0x78},
        {0x3c,0x66,0x66,0x66,0x3c,0x18,0x7e,0x18},
        {0x3f,0x33,0x3f,0x30,0x30,0x70,0xf0,0xe0},
        {0x7f,0x63,0x7f,0x63,0x63,0x67,0xe6,0xc0},
        {0x99,0x5a,0x3c,0xe7,0xe7,0x3c,0x5a,0x99},
        {0x80,0xe0,0xf8,0xfe,0xf8,0xe0,0x80,0x00},
        {0x02,0x0e,0x3e,0xfe,0x3e,0x0e,0x02,0x00},
        {0x18,0x3c,0x7e,0x18,0x18,0x7e,0x3c,0x18},
        {0x66,0x66,0x66,0x66,0x66,0x00,0x66,0x00},
        {0x7f,0xdb,0xdb,0x7b,0x1b,0x1b,0x1b,0x00},
        {0x3e,0x63,0x38,0x6c,0x6c,0x38,0xcc,0x78},
        {0x00,0x00,0x00,0x00,0x7e,0x7e,0x7e,0x00},
        {0x18,0x3c,0x7e,0x18,0x7e,0x3c,0x18,0x7e},
        {0x18,0x3c,0x7e,0x18,0x18,0x18,0x18,0x00},
        {0x18,0x18,0x18,0x18,0x7e,0x3c,0x18,0x00},
        {0x00,0x18,0x0c,0xfe,0x0c,0x18,0x00,0x00},
        {0x00,0x30,0x60,0xfe,0x60,0x30,0x00,0x00},
        {0x00,0x00,0xc0,0xc0,0xc0,0xfe,0x00,0x00},
        {0x00,0x24,0x66,0xff,0x66,0x24,0x00,0x00},
        {0x00,0x18,0x3c,0x7e,0xff,0xff,0x00,0x00},
        {0x00,0xff,0xff,0x7e,0x3c,0x18,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x18,0x3c,0x3c,0x18,0x18,0x00,0x18,0x00},
        {0x6c,0x6c,0x6c,0x00,0x00,0x00,0x00,0x00},
        {0x6c,0x6c,0xfe,0x6c,0xfe,0x6c,0x6c,0x00},
        {0x18,0x3e,0x60,0x3c,0x06,0x7c,0x18,0x00},
        {0x00,0xc6,0xcc,0x18,0x30,0x66,0xc6,0x00},
        {0x38,0x6c,0x38,0x76,0xdc,0xcc,0x76,0x00},
        {0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00},
        {0x0c,0x18,0x30,0x30,0x30,0x18,0x0c,0x00},
        {0x30,0x18,0x0c,0x0c,0x0c,0x18,0x30,0x00},
        {0x00,0x66,0x3c,0xff,0x3c,0x66,0x00,0x00},
        {0x00,0x18,0x18,0x7e,0x18,0x18,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30},
        {0x00,0x00,0x00,0x7e,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00},
        {0x06,0x0c,0x18,0x30,0x60,0xc0,0x80,0x00},
        {0x7c,0xc6,0xce,0xde,0xf6,0xe6,0x7c,0x00},
        {0x18,0x38,0x18,0x18,0x18,0x18,0x7e,0x00},
        {0x7c,0xc6,0x0e,0x1c,0x38,0x70,0xfe,0x00},
        {0x7c,0xc6,0x06,0x3c,0x06,0xc6,0x7c,0x00},
        {0x1c,0x3c,0x6c,0xcc,0xfe,0x0c,0x1e,0x00},
        {0xfe,0xc0,0xfc,0x06,0x06,0xc6,0x7c,0x00},
        {0x3c,0x60,0xc0,0xfc,0xc6,0xc6,0x7c,0x00},
        {0xfe,0xc6,0x0c,0x18,0x30,0x30,0x30,0x00},
        {0x7c,0xc6,0xc6,0x7c,0xc6,0xc6,0x7c,0x00},
        {0x7c,0xc6,0xc6,0x7e,0x06,0x0c,0x78,0x00},
        {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00},
        {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30},
        {0x0e,0x1c,0x38,0x70,0x38,0x1c,0x0e,0x00},
        {0x00,0x00,0x7e,0x00,0x00,0x7e,0x00,0x00},
        {0x70,0x38,0x1c,0x0e,0x1c,0x38,0x70,0x00},
        {0x7c,0xc6,0x0e,0x1c,0x18,0x00,0x18,0x00},
        {0x7c,0xc6,0xde,0xde,0xde,0xc0,0x7c,0x00},
        {0x38,0x6c,0xc6,0xc6,0xfe,0xc6,0xc6,0x00},
        {0xfc,0x66,0x66,0x7c,0x66,0x66,0xfc,0x00},
        {0x3c,0x66,0xc0,0xc0,0xc0,0x66,0x3c,0x00},
        {0xf8,0x6c,0x66,0x66,0x66,0x6c,0xf8,0x00},
        {0xfe,0x62,0x68,0x78,0x68,0x62,0xfe,0x00},
        {0xfe,0x62,0x68,0x78,0x68,0x60,0xf0,0x00},
        {0x3c,0x66,0xc0,0xc0,0xce,0x66,0x3e,0x00},
        {0xc6,0xc6,0xc6,0xfe,0xc6,0xc6,0xc6,0x00},
        {0x3c,0x18,0x18,0x18,0x18,0x18,0x3c,0x00},
        {0x1e,0x0c,0x0c,0x0c,0xcc,0xcc,0x78,0x00},
        {0xe6,0x66,0x6c,0x78,0x6c,0x66,0xe6,0x00},
        {0xf0,0x60,0x60,0x60,0x62,0x66,0xfe,0x00},
        {0xc6,0xee,0xfe,0xd6,0xc6,0xc6,0xc6,0x00},
        {0xc6,0xe6,0xf6,0xde,0xce,0xc6,0xc6,0x00},
        {0x7c,0xc6,0xc6,0xc6,0xc6,0xc6,0x7c,0x00},
        {0xfc,0x66,0x66,0x7c,0x60,0x60,0xf0,0x00},
        {0x7c,0xc6,0xc6,0xc6,0xd6,0xcc,0x76,0x00},
        {0xfc,0x66,0x66,0x7c,0x6c,0x66,0xe6,0x00},
        {0x7c,0xc6,0x60,0x38,0x0c,0xc6,0x7c,0x00},
        {0x7e,0x7e,0x5a,0x18,0x18,0x18,0x3c,0x00},
        {0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0x7c,0x00},
        {0xc6,0xc6,0xc6,0xc6,0xc6,0x6c,0x38,0x00},
        {0xc6,0xc6,0xc6,0xd6,0xfe,0xee,0xc6,0x00},
        {0xc6,0xc6,0x6c,0x38,0x6c,0xc6,0xc6,0x00},
        {0x66,0x66,0x66,0x3c,0x18,0x18,0x3c,0x00},
        {0xfe,0xc6,0x8c,0x18,0x32,0x66,0xfe,0x00},
        {0x3c,0x30,0x30,0x30,0x30,0x30,0x3c,0x00},
        {0xc0,0x60,0x30,0x18,0x0c,0x06,0x02,0x00},
        {0x3c,0x0c,0x0c,0x0c,0x0c,0x0c,0x3c,0x00},
        {0x10,0x38,0x6c,0xc6,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff},
        {0x30,0x18,0x0c,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x7c,0x06,0x7e,0xc6,0x7e,0x00},
        {0xe0,0x60,0x7c,0x66,0x66,0x66,0xdc,0x00},
        {0x00,0x00,0x7c,0xc6,0xc0,0xc6,0x7c,0x00},
        {0x1c,0x0c,0x7c,0xcc,0xcc,0xcc,0x7e,0x00},
        {0x00,0x00,0x7c,0xc6,0xfe,0xc0,0x7c,0x00},
        {0x3c,0x66,0x60,0xf8,0x60,0x60,0xf0,0x00},
        {0x00,0x00,0x7e,0xcc,0xcc,0x7c,0x0c,0xf8},
        {0xe0,0x60,0x6c,0x76,0x66,0x66,0xe6,0x00},
        {0x18,0x00,0x38,0x18,0x18,0x18,0x3c,0x00},
        {0x06,0x00,0x06,0x06,0x06,0x66,0x66,0x3c},
        {0xe0,0x60,0x66,0x6c,0x78,0x6c,0xe6,0x00},
        {0x38,0x18,0x18,0x18,0x18,0x18,0x3c,0x00},
        {0x00,0x00,0xec,0xfe,0xd6,0xd6,0xc6,0x00},
        {0x00,0x00,0xdc,0x66,0x66,0x66,0x66,0x00},
        {0x00,0x00,0x7c,0xc6,0xc6,0xc6,0x7c,0x00},
        {0x00,0x00,0xdc,0x66,0x66,0x7c,0x60,0xf0},
        {0x00,0x00,0x7e,0xcc,0xcc,0x7c,0x0c,0x1e},
        {0x00,0x00,0xdc,0x76,0x66,0x60,0xf0,0x00},
        {0x00,0x00,0x7e,0xc0,0x7c,0x06,0xfc,0x00},
        {0x10,0x30,0x7c,0x30,0x30,0x36,0x1c,0x00},
        {0x00,0x00,0xcc,0xcc,0xcc,0xcc,0x7e,0x00},
        {0x00,0x00,0xc6,0xc6,0xc6,0x6c,0x38,0x00},
        {0x00,0x00,0xc6,0xd6,0xd6,0xfe,0x6c,0x00},
        {0x00,0x00,0xc6,0x6c,0x38,0x6c,0xc6,0x00},
        {0x00,0x00,0xc6,0xc6,0xc6,0x7e,0x06,0xfc},
        {0x00,0x00,0xfe,0x8c,0x18,0x32,0xfe,0x00},
        {0x0e,0x18,0x18,0x70,0x18,0x18,0x0e,0x00},
        {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00},
        {0x70,0x18,0x18,0x0e,0x18,0x18,0x70,0x00},
        {0x76,0xdc,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
};

typedef struct {
	uint8_t hue;
	uint8_t brightness;
	uint8_t wave_depth;
	bool invert;
} sid_visual_state_t;

typedef struct {
	uint32_t magic;
	uint32_t count;
	uint32_t frame;
} sid_header_t;

typedef struct {
	uint8_t chip;
	uint8_t addr;
	uint8_t value;
	uint8_t pad;
	uint32_t delta;
} sid_event_t;

static struct mutex frame_logic_mutex;
static critical_section_t sid_state_lock;
static volatile uint32_t frame_counter = 0;

static sid_visual_state_t visual_state = {
	.hue = 48,
	.brightness = 32,
	.wave_depth = 16,
	.invert = false,
};

static uint32_t sid_last_activity = 0;

static uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
static size_t serial_buffer_len = 0;
static bool have_header = false;
static sid_header_t current_header;
static uint32_t events_remaining = 0;
static uint32_t frame_event_count = 0;
static uint32_t frame_value_accum = 0;
static uint8_t frame_last_chip = 0;
static bool cdc_online = false;
static bool cdc_port_open = false;
static bool cdc_suspended = false;
static bool audio_ready = false;
static uint32_t last_sid_frame_index = 0;
static bool led_state = false;
static char status_lines[TEXT_ROWS][TEXT_COLS];
static bool sid_frame_offset_valid = false;
static int32_t sid_frame_offset = 0;

typedef struct {
	uint8_t chip_mask;
	uint8_t addr;
	uint8_t value;
	uint16_t delta;
} event_log_entry_t;

static event_log_entry_t frame_event_log[EVENT_LOG_MAX];
static uint32_t frame_event_log_count = 0;
static bool frame_event_log_overflow = false;

static event_log_entry_t last_event_log[EVENT_LOG_MAX];
static uint32_t last_event_log_count = 0;
static bool last_event_log_overflow = false;
static uint32_t last_frame_event_total = 0;
static uint32_t last_chip_events[2];
static bool last_event_log_ready = false;

typedef struct {
	uint64_t frames;
	uint64_t events_total;
	uint32_t max_events;
	uint32_t last_events;
	uint64_t bytes_total;
	uint32_t max_bytes;
	uint32_t last_bytes;
	uint64_t frame_time_total;
	uint32_t last_frame_us;
	uint32_t max_frame_us;
	uint64_t frame_gap_total;
	uint32_t last_frame_gap_us;
	uint32_t max_frame_gap_us;
	uint64_t cdc_bytes_total;
	uint32_t max_cdc_bytes;
	uint32_t last_cdc_bytes;
	uint32_t last_cdc_rate_kbps;
	uint32_t max_cdc_rate_kbps;
	uint32_t cdc_reads_total;
	uint32_t last_cdc_reads;
	uint64_t parse_time_total;
	uint64_t parse_samples;
	uint32_t last_parse_us;
	uint32_t max_parse_us;
	int32_t last_frame_drift;
	int32_t max_frame_drift;
	size_t buffer_peak;
} sid_stats_t;

static sid_stats_t sid_stats;
static uint32_t current_frame_bytes = 0;
static uint32_t frame_start_us = 0;
static uint32_t last_frame_complete_us = 0;

static inline uint16_t pack_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t) ((r << 11) | (g << 5) | b);
}

static inline uint16_t rgb_from_u8(uint8_t r8, uint8_t g8, uint8_t b8) {
	return pack_rgb565(r8 >> 3, g8 >> 2, b8 >> 3);
}

static inline int hex_value(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static inline int parse_hex_byte(char hi, char lo) {
	int h = hex_value(hi);
	int l = hex_value(lo);
	if (h < 0 || l < 0) return -1;
	return (h << 4) | l;
}

static inline int32_t abs_i32(int32_t v) {
	return (v < 0) ? -v : v;
}

static void clear_status_lines(void) {
    critical_section_enter_blocking(&sid_state_lock);
    for (int row = 0; row < TEXT_ROWS; ++row) {
        memset(status_lines[row], ' ', TEXT_COLS);
    }
    critical_section_exit(&sid_state_lock);
}
static void set_status_line(int row, const char *fmt, ...) {
	if (row < 0 || row >= TEXT_ROWS) {
		return;
	}
	char temp[TEXT_COLS + 1];
	va_list args;
	va_start(args, fmt);
	int written = vsnprintf(temp, sizeof temp, fmt, args);
	va_end(args);
	if (written < 0) {
		written = 0;
	}
	size_t len = (size_t) written;
	if (len > TEXT_COLS) {
		len = TEXT_COLS;
	}
	critical_section_enter_blocking(&sid_state_lock);
	memset(status_lines[row], ' ', TEXT_COLS);
	memcpy(status_lines[row], temp, len);
	critical_section_exit(&sid_state_lock);
}

static void update_cdc_status_line(const char *cdc_state) {
	const char *audio_state = audio_ready ? "AUD OK" : "AUD ERR";
	set_status_line(3, "%-10s | %-7s", cdc_state, audio_state);
}

static void format_event_entry(char *dst, size_t dst_len, event_log_entry_t const *entry) {
	unsigned delta = (unsigned) entry->delta;
	/* Fixed-width delta: D followed by two hex digits, or DFF+ if over 0xFF */
	if (delta > 0xFFu) {
		snprintf(dst, dst_len, "D++ $%02X=#$%02X",
		         entry->addr & 0x1Fu,
		         entry->value);
		return;
	}
	snprintf(dst, dst_len, "D%02X $%02X=#$%02X",
	         delta,
	         entry->addr & 0x1Fu,
	         entry->value);
}

static void update_event_trace_display(uint32_t sid_frame_index, uint32_t video_frame_index) {
	const unsigned cols = 3;
	const unsigned col_width = 18;
	size_t row = EVENT_ROW_START;
	if (!last_event_log_ready) {
		set_status_line((int) row++, "Waiting for SID data...");
		while (row < TEXT_ROWS) {
			set_status_line((int) row++, "");
		}
		return;
	}

	const char *overflow = last_event_log_overflow ? "+" : " ";
	unsigned total_events = (unsigned) ((last_frame_event_total > 999u) ? 999u : last_frame_event_total);
	char header[TEXT_COLS + 1];
	snprintf(header, sizeof header, "SID %08lu VF %08lu EV%03u%s DF%+04ld",
	         (unsigned long) sid_frame_index,
	         (unsigned long) video_frame_index,
	         total_events,
	         overflow,
	         (long) sid_stats.last_frame_drift);
	set_status_line((int) row++, header);

	char chip_line[TEXT_COLS + 1];
	snprintf(chip_line, sizeof chip_line, "  C0:%02u  C1:%02u  Total:%03u",
	         (unsigned) last_chip_events[0],
	         (unsigned) last_chip_events[1],
	         total_events);
	set_status_line((int) row++, chip_line);

	if (last_frame_event_total == 0) {
		set_status_line((int) row++, "  (no SID writes this frame)");
		while (row < TEXT_ROWS) {
			set_status_line((int) row++, "");
		}
		return;
	}

	uint32_t idx = 0;
	while (idx < last_event_log_count && row < TEXT_ROWS) {
		char line[TEXT_COLS + 1];
		char *p = line;
		size_t remaining = TEXT_COLS;
		for (unsigned c = 0; c < cols && idx < last_event_log_count; ++c, ++idx) {
			char entry[20];
			format_event_entry(entry, sizeof entry, &last_event_log[idx]);
			int written = snprintf(p, remaining, "%-*.*s",
			                       (int) col_width, (int) col_width, entry);
			if (written < 0) written = 0;
			if ((size_t) written > remaining) written = (int) remaining;
			p += written;
			remaining = (written < (int) remaining) ? remaining - (size_t) written : 0;
		}
		while (remaining > 0) {
			*p++ = ' ';
			--remaining;
		}
		line[TEXT_COLS] = '\0';
		set_status_line((int) row++, line);
	}

	if (last_event_log_overflow && row < TEXT_ROWS) {
		unsigned more = (last_frame_event_total > last_event_log_count)
		                 ? (unsigned) (last_frame_event_total - last_event_log_count)
		                 : 0u;
		char overflow_line[TEXT_COLS + 1];
		snprintf(overflow_line, sizeof overflow_line, "  ... %u more event%s",
		         more, (more == 1u) ? "" : "s");
		set_status_line((int) row++, overflow_line);
	}

	while (row < TEXT_ROWS) {
		set_status_line((int) row++, "");
	}
}

static uint16_t *__time_critical_func(raw_scanline_prepare)(struct scanvideo_scanline_buffer *dest, uint width) {
	assert(width >= 3);
	assert((width & 1u) == 0);
	dest->data[0] = COMPOSABLE_RAW_RUN | ((width + 1u - 3u) << 16u);
	dest->data[width / 2u + 2u] = (uint32_t) COMPOSABLE_EOL_ALIGN << 16u;
	dest->data_used = width / 2u + 2u;
	return (uint16_t *) &dest->data[1];
}

static void __time_critical_func(raw_scanline_finish)(struct scanvideo_scanline_buffer *dest) {
	uint32_t first = dest->data[0];
	uint32_t second = dest->data[1];
	dest->data[0] = (first & 0x0000ffffu) | ((second & 0x0000ffffu) << 16u);
	dest->data[1] = (second & 0xffff0000u) | ((first & 0xffff0000u) >> 16u);
	dest->status = SCANLINE_OK;
}
static void __time_critical_func(frame_update_logic)(void);
static void __time_critical_func(render_scanline)(struct scanvideo_scanline_buffer *dest, int core);
static void __time_critical_func(render_loop)(void);
static uint16_t __time_critical_func(compute_line_color)(int scanline, uint32_t frame,
                                                         sid_visual_state_t const *state);
static void process_serial_stream(void);
static void complete_sid_frame(uint32_t frame_index);

static void complete_sid_frame(uint32_t frame_index) {
	uint32_t events = frame_event_count;
	uint32_t value_sum = frame_value_accum;
	uint8_t last_chip = frame_last_chip;

	frame_event_count = 0;
	frame_value_accum = 0;
	frame_last_chip = 0;

	uint32_t video_frame = frame_counter;

	uint32_t end_us = time_us_32();
	uint32_t frame_duration = frame_start_us ? (uint32_t) (end_us - frame_start_us) : 0;
	sid_stats.last_frame_us = frame_duration;
	sid_stats.frame_time_total += frame_duration;
	if (frame_duration > sid_stats.max_frame_us) sid_stats.max_frame_us = frame_duration;

	uint32_t frame_gap_us = 0;
	if (last_frame_complete_us != 0 && frame_start_us != 0) {
		frame_gap_us = (uint32_t) (frame_start_us - last_frame_complete_us);
		sid_stats.frame_gap_total += frame_gap_us;
		if (frame_gap_us > sid_stats.max_frame_gap_us) sid_stats.max_frame_gap_us = frame_gap_us;
	}
	sid_stats.last_frame_gap_us = frame_gap_us;
	last_frame_complete_us = end_us;
	frame_start_us = 0;

	uint8_t new_brightness = (uint8_t) (events * 8u);
	if (new_brightness < 32u) {
		new_brightness += 32u;
	}
	if (new_brightness > 240u) {
		new_brightness = 240u;
	}

	uint8_t avg_value = (events > 0) ? (uint8_t) (value_sum / events) : 0u;
	uint8_t new_wave = (uint8_t) (avg_value * 3u);
	if (new_wave < 12u) {
		new_wave = 12u;
	}

	size_t buffer_fill;
	bool connected_state;

	critical_section_enter_blocking(&sid_state_lock);

	visual_state.hue = (uint8_t) frame_index;
	visual_state.brightness = new_brightness;
	visual_state.wave_depth = new_wave;
	visual_state.invert = (last_chip & 0x01u) != 0u;
	sid_last_activity = frame_counter;
	last_sid_frame_index = frame_index;
	buffer_fill = serial_buffer_len;
	connected_state = cdc_online;

	critical_section_exit(&sid_state_lock);

	if (!sid_frame_offset_valid) {
		sid_frame_offset = (int32_t) video_frame - (int32_t) frame_index;
		sid_frame_offset_valid = true;
	}
	int32_t drift = (int32_t) video_frame - (int32_t) frame_index - sid_frame_offset;
	sid_stats.last_frame_drift = drift;
	if (abs_i32(drift) > abs_i32(sid_stats.max_frame_drift)) {
		sid_stats.max_frame_drift = drift;
	}

	uint64_t frame_count = ++sid_stats.frames;
	sid_stats.last_events = events;
	sid_stats.events_total += events;
	if (events > sid_stats.max_events) sid_stats.max_events = events;
	sid_stats.last_bytes = current_frame_bytes;
	sid_stats.bytes_total += current_frame_bytes;
	if (current_frame_bytes > sid_stats.max_bytes) sid_stats.max_bytes = current_frame_bytes;
	if (buffer_fill > sid_stats.buffer_peak) sid_stats.buffer_peak = buffer_fill;
	uint32_t avg_events = frame_count ? (uint32_t) (sid_stats.events_total / frame_count) : 0;
	uint32_t avg_bytes = frame_count ? (uint32_t) (sid_stats.bytes_total / frame_count) : 0;
	uint32_t avg_cdc_bytes = frame_count ? (uint32_t) (sid_stats.cdc_bytes_total / frame_count) : 0;
	uint32_t avg_frame_us = frame_count ? (uint32_t) (sid_stats.frame_time_total / frame_count) : 0;
	uint32_t avg_frame_gap_us = (frame_count > 1)
	                               ? (uint32_t) (sid_stats.frame_gap_total / (frame_count - 1))
	                               : 0;
	uint32_t avg_parse_us = sid_stats.parse_samples
	                            ? (uint32_t) (sid_stats.parse_time_total / sid_stats.parse_samples)
	                            : 0;
	uint32_t avg_cdc_rate = sid_stats.parse_time_total
	                            ? (uint32_t) ((sid_stats.cdc_bytes_total * 1000ull) / sid_stats.parse_time_total)
	                            : 0;
	current_frame_bytes = 0;

	led_state = !led_state;
	gpio_put(PICO_DEFAULT_LED_PIN, led_state);

	set_status_line(1, "FRAME %08lu", (unsigned long) frame_index);
	set_status_line(2, "EVENTS %3u AVG %3u CHIP %-2u", events, avg_events, last_chip & 0x03u);
	set_status_line(4, "BUFFER %4zu CDC %s", buffer_fill, connected_state ? "ON" : "OFF");
	set_status_line(LOG_ROW_START + 0, "Frames %10llu | Ev L%3u A%3u M%3u",
	               (unsigned long long) frame_count, sid_stats.last_events, avg_events, sid_stats.max_events);
	set_status_line(LOG_ROW_START + 1, "Bytes  %10llu | L%3u A%3u M%3u",
	               (unsigned long long) sid_stats.bytes_total, sid_stats.last_bytes, avg_bytes, sid_stats.max_bytes);
	set_status_line(LOG_ROW_START + 2, "Frame us L%4u A%4u M%4u",
	               sid_stats.last_frame_us, avg_frame_us, sid_stats.max_frame_us);
	set_status_line(LOG_ROW_START + 3, "Gap   us L%4u A%4u M%4u",
	               sid_stats.last_frame_gap_us, avg_frame_gap_us, sid_stats.max_frame_gap_us);
	set_status_line(LOG_ROW_START + 4, "Drift    L%+4ld M%+4ld",
	               (long) sid_stats.last_frame_drift, (long) sid_stats.max_frame_drift);
	set_status_line(LOG_ROW_START + 5, "CDC bytes L%4u A%4u M%4u",
	               sid_stats.last_cdc_bytes, avg_cdc_bytes, sid_stats.max_cdc_bytes);
	set_status_line(LOG_ROW_START + 6, "CDC rate  L%4u A%4u M%4u",
	               sid_stats.last_cdc_rate_kbps, avg_cdc_rate, sid_stats.max_cdc_rate_kbps);
	set_status_line(LOG_ROW_START + 7, "CDC reads L%3u Tot %6u",
	               sid_stats.last_cdc_reads, sid_stats.cdc_reads_total);
	set_status_line(LOG_ROW_START + 8, "Parse us L%4u A%4u M%4u",
	               sid_stats.last_parse_us, avg_parse_us, sid_stats.max_parse_us);
	set_status_line(LOG_ROW_START + 9, "Buffer   now %4zu pk %4zu",
	               buffer_fill, sid_stats.buffer_peak);

	last_event_log_count = frame_event_log_count;
	if (last_event_log_count > EVENT_LOG_MAX) {
		last_event_log_count = EVENT_LOG_MAX;
	}
	if (last_event_log_count > 0) {
		memcpy(last_event_log, frame_event_log,
		       last_event_log_count * sizeof(event_log_entry_t));
	}
	last_event_log_overflow = frame_event_log_overflow || (events > last_event_log_count);
	last_frame_event_total = events;
	memset(last_chip_events, 0, sizeof last_chip_events);
	for (uint32_t i = 0; i < last_event_log_count; ++i) {
		uint8_t mask = last_event_log[i].chip_mask & 0x03u;
		if (mask & 0x01u) last_chip_events[0]++;
		if (mask & 0x02u) last_chip_events[1]++;
	}
	last_event_log_ready = true;
	update_event_trace_display(frame_index, video_frame);
}

static void process_serial_stream(void) {
	uint32_t parse_start = time_us_32();
	uint32_t bytes_read_total = 0;
	uint32_t read_count = 0;
	tud_task();
	bool mounted = tud_mounted();
	bool suspended = tud_suspended();
	bool port_open = tud_cdc_connected();
	bool changed = (mounted != cdc_online) || (port_open != cdc_port_open) || (suspended != cdc_suspended);

	if (changed) {
		critical_section_enter_blocking(&sid_state_lock);
		if (!mounted || suspended || !port_open) {
			visual_state.invert = false;
			visual_state.brightness = 20;
			visual_state.wave_depth = 16;
		}
		cdc_online = mounted;
		cdc_port_open = port_open;
		cdc_suspended = suspended;
		sid_last_activity = frame_counter;
		critical_section_exit(&sid_state_lock);

		const char *status;
		if (!mounted) {
			status = "CDC WAITING";
		} else if (suspended) {
			status = "CDC SUSP";
		} else if (!port_open) {
			status = "CDC IDLE";
		} else {
			status = "CDC READY";
		}
		update_cdc_status_line(status);
	}

	if (!mounted) {
		serial_buffer_len = 0;
		have_header = false;
		events_remaining = 0;
		frame_event_count = 0;
		frame_value_accum = 0;
		frame_last_chip = 0;
		sid_frame_offset_valid = false;
		sid_frame_offset = 0;
		frame_event_log_count = 0;
		frame_event_log_overflow = false;
		last_event_log_count = 0;
		last_event_log_overflow = false;
		last_frame_event_total = 0;
		memset(last_chip_events, 0, sizeof last_chip_events);
		last_event_log_ready = false;
		update_event_trace_display(0, frame_counter);
		set_status_line(4, "BUFFER    0 CDC OFF");
		if (audio_ready) {
			siddler_audio_reset_state();
		}
		return;
	}

	uint8_t usb_chunk[CFG_TUD_CDC_EP_BUFSIZE];
	while (tud_cdc_available()) {
		uint32_t count = tud_cdc_read(usb_chunk, sizeof usb_chunk);
		if (!count) {
			break;
		}
		for (uint32_t i = 0; i < count; ++i) {
			if (serial_buffer_len == SERIAL_BUFFER_SIZE) {
				size_t drop = SERIAL_BUFFER_SIZE / 2;
				memmove(serial_buffer, serial_buffer + drop, SERIAL_BUFFER_SIZE - drop);
				serial_buffer_len -= drop;
			}
			serial_buffer[serial_buffer_len++] = usb_chunk[i];
		}
		bytes_read_total += count;
		read_count++;
	}

	uint32_t duration = time_us_32() - parse_start;
	sid_stats.last_parse_us = duration;
	if (duration > sid_stats.max_parse_us) sid_stats.max_parse_us = duration;
	sid_stats.parse_time_total += duration;
	sid_stats.parse_samples++;
	sid_stats.last_cdc_bytes = bytes_read_total;
	sid_stats.cdc_bytes_total += bytes_read_total;
	if (bytes_read_total > sid_stats.max_cdc_bytes) sid_stats.max_cdc_bytes = bytes_read_total;
	uint32_t last_cdc_rate = 0;
	if (duration > 0 && bytes_read_total > 0) {
		last_cdc_rate = (uint32_t) (((uint64_t) bytes_read_total * 1000ull) / duration);
	}
	sid_stats.last_cdc_rate_kbps = last_cdc_rate;
	if (last_cdc_rate > sid_stats.max_cdc_rate_kbps) sid_stats.max_cdc_rate_kbps = last_cdc_rate;
	sid_stats.last_cdc_reads = read_count;
	sid_stats.cdc_reads_total += read_count;

	size_t offset = 0;

	while (offset < serial_buffer_len) {
		size_t remaining = serial_buffer_len - offset;

		if (!have_header) {
			if (remaining < sizeof(sid_header_t)) {
				break;
			}
			memcpy(&current_header, serial_buffer + offset, sizeof(sid_header_t));
			if (current_header.magic != SID_MAGIC) {
				offset += 1;
				continue;
			}
			if (current_header.count > 8192u) {
				offset += 1;
				continue;
			}
			have_header = true;
			frame_start_us = time_us_32();
			events_remaining = current_header.count;
			frame_event_count = 0;
			frame_value_accum = 0;
			frame_last_chip = 0;
			frame_event_log_count = 0;
			frame_event_log_overflow = false;
			current_frame_bytes = sizeof(sid_header_t);
			offset += sizeof(sid_header_t);
			continue;
		}

		if (events_remaining == 0) {
			complete_sid_frame(current_header.frame);
			have_header = false;
			continue;
		}

		if (remaining < sizeof(sid_event_t)) {
			break;
		}

		sid_event_t event;
		memcpy(&event, serial_buffer + offset, sizeof(sid_event_t));
		offset += sizeof(sid_event_t);
		if (events_remaining > 0) {
			events_remaining--;
		}

		frame_event_count++;
		frame_value_accum += event.value;
		current_frame_bytes += sizeof(sid_event_t);
		frame_last_chip = event.chip;
		siddler_audio_queue_event(event.chip, event.addr, event.value, event.delta);
		if (audio_ready) {
			siddler_audio_task();
		}
		uint8_t chip_mask = event.chip & 0x03u;
		uint16_t delta16 = (frame_event_log_count == 0)
				? 0u
				: ((event.delta > 0xFFFFu) ? 0xFFFFu : (uint16_t) event.delta);
		if (frame_event_log_count < EVENT_LOG_MAX) {
			frame_event_log[frame_event_log_count++] = (event_log_entry_t) {
				.chip_mask = chip_mask,
				.addr = event.addr,
				.value = event.value,
				.delta = delta16,
			};
		} else {
			frame_event_log_overflow = true;
		}

		if (events_remaining == 0) {
			complete_sid_frame(current_header.frame);
			have_header = false;
		}
	}

	if (offset > 0) {
		serial_buffer_len -= offset;
		memmove(serial_buffer, serial_buffer + offset, serial_buffer_len);
	}

	const char *buf_status = port_open ? "ON" : "IDLE";
	set_status_line(4, "BUFFER %4zu CDC %s", serial_buffer_len, buf_status);
	if (audio_ready) {
		siddler_audio_task();
	}
}

static void __time_critical_func(frame_update_logic)(void) {
	frame_counter++;

	critical_section_enter_blocking(&sid_state_lock);
	uint32_t inactivity = frame_counter - sid_last_activity;
	if (inactivity > 120u) {
		if (visual_state.brightness > 12u) {
			visual_state.brightness--;
		}
		if (visual_state.wave_depth > 8u) {
			visual_state.wave_depth--;
		}
	}
	if (inactivity > 360u) {
		visual_state.hue++;
	}
	critical_section_exit(&sid_state_lock);
    set_status_line(5, "FRAME %08lu 50Hz|NoMore KRRZPRRTs", (unsigned long) frame_counter);
	if ((frame_counter - last_sid_frame_index) > 300u && (frame_counter & 0x3Fu) == 0u) {
		set_status_line(1, "WAITING FOR SID DATA");
	}
}

static uint16_t __time_critical_func(compute_line_color)(int scanline, uint32_t frame, sid_visual_state_t const *state) {
	int x = scanline;
	int y = (int) frame;
	int t = (int) (state->hue);
	int v = (int) (state->wave_depth);

	int a = (int) (sinf((float) (x + t) * 0.045f) * 31.0f + 31.0f);
	int b = (int) (sinf((float) (y + v) * 0.055f) * 31.0f + 31.0f);
	int c = (int) (sinf((float) (x + y + t) * 0.032f) * 31.0f + 31.0f);

	uint8_t r = (uint8_t) ((a + c) & 0x1Fu);
	uint8_t g = (uint8_t) (((a + b) * 2) & 0x3Fu);
	uint8_t blue = (uint8_t) ((b + c) & 0x1Fu);

	uint32_t brightness = (uint32_t) state->brightness + 32u;
	if (brightness > 255u) brightness = 255u;
	r = (uint8_t) ((r * brightness) >> 8);
	g = (uint8_t) ((g * brightness) >> 8);
	blue = (uint8_t) ((blue * brightness) >> 8);

	if (state->invert) {
		r = (uint8_t) (0x1Fu - r);
		g = (uint8_t) (0x3Fu - g);
		blue = (uint8_t) (0x1Fu - blue);
	}

	return pack_rgb565(r, g, blue);
}

static void __time_critical_func(render_scanline)(struct scanvideo_scanline_buffer *dest, int core) {
	(void) core;
	int scanline = scanvideo_scanline_number(dest->scanline_id);
	int text_row = scanline / CHAR_HEIGHT;
	int glyph_row = scanline % CHAR_HEIGHT;

	sid_visual_state_t local_state;
	char row_chars[TEXT_COLS];
	bool row_valid = false;

	critical_section_enter_blocking(&sid_state_lock);
	local_state = visual_state;
	if (text_row >= 0 && text_row < TEXT_ROWS) {
		memcpy(row_chars, status_lines[text_row], TEXT_COLS);
		row_valid = true;
	}
	critical_section_exit(&sid_state_lock);

	uint16_t *pixels = raw_scanline_prepare(dest, VGA_MODE.width);
	uint32_t frame = frame_counter;
	uint16_t bg_color = compute_line_color(scanline, frame, &local_state);

	uint16_t char_colors[TEXT_COLS];
	for (int c = 0; c < TEXT_COLS; ++c) {
		char_colors[c] = 0xffffu;
	}

	if (row_valid) {
		if (text_row == 0) {
			uint16_t title_color = rgb_from_u8(255, 210, 80);
			for (int c = 0; c < TEXT_COLS; ++c) char_colors[c] = title_color;
		} else if (text_row == 1 || text_row == 2) {
			uint16_t status_color = rgb_from_u8(190, 230, 255);
			for (int c = 0; c < TEXT_COLS; ++c) char_colors[c] = status_color;
		} else if (text_row == 3) {
			uint16_t cdc_color = rgb_from_u8(190, 255, 210);
			for (int c = 0; c < TEXT_COLS; ++c) char_colors[c] = cdc_color;
		} else if (text_row == 4) {
			uint16_t buffer_color = rgb_from_u8(255, 225, 200);
			for (int c = 0; c < TEXT_COLS; ++c) char_colors[c] = buffer_color;
		} else if (text_row == 5) {
			uint16_t frame_color = rgb_from_u8(240, 200, 255);
			for (int c = 0; c < TEXT_COLS; ++c) char_colors[c] = frame_color;
		}

		if (text_row >= EVENT_ROW_START && row_chars[0] == 'D') {
			uint16_t dark_gray = rgb_from_u8(100, 100, 100);
			uint16_t address_color = rgb_from_u8(255, 240, 80);
			int idx = 0;
			while (idx < TEXT_COLS) {
				char ch = row_chars[idx];
				if (ch == ' ') break;
				char_colors[idx] = dark_gray;
				idx++;
			}
			while (idx < TEXT_COLS && row_chars[idx] == ' ') idx++;
			if (idx < TEXT_COLS && row_chars[idx] == '$') {
				char_colors[idx] = dark_gray;
				idx++;
				for (int j = 0; j < 2 && idx < TEXT_COLS; ++j, ++idx) {
					if (isxdigit((unsigned char) row_chars[idx])) char_colors[idx] = address_color;
				}
			}
			if (idx < TEXT_COLS && row_chars[idx] == '=') {
				char_colors[idx] = dark_gray;
				idx++;
			}
			if (idx < TEXT_COLS && row_chars[idx] == '#') {
				char_colors[idx] = dark_gray;
				idx++;
			}
			if (idx < TEXT_COLS && row_chars[idx] == '$') {
				char_colors[idx] = dark_gray;
				idx++;
			}
			if (idx + 1 < TEXT_COLS && isxdigit((unsigned char) row_chars[idx]) && isxdigit((unsigned char) row_chars[idx + 1])) {
				int value = parse_hex_byte(row_chars[idx], row_chars[idx + 1]);
				if (value >= 0) {
					uint8_t v = (uint8_t) value;
					uint8_t r8 = v;
					int diff = v - 128;
					if (diff < 0) diff = -diff;
					int g = 255 - diff * 2;
					if (g < 0) g = 0;
					uint8_t g8 = (uint8_t) g;
					uint8_t b8 = (uint8_t) (255 - v);
					uint16_t value_color = rgb_from_u8(r8, g8, b8);
					char_colors[idx] = value_color;
					char_colors[idx + 1] = value_color;
				}
				idx += 2;
			}
		}
	}

	if (!row_valid) {
		for (int x = 0; x < VGA_MODE.width; ++x) {
			pixels[x] = bg_color;
		}
		raw_scanline_finish(dest);
		return;
	}

	for (int x = 0; x < VGA_MODE.width; ++x) {
		int col = x / CHAR_WIDTH;
		uint16_t color = bg_color;
		if (col < TEXT_COLS) {
			uint8_t ch = (uint8_t) row_chars[col];
			uint8_t glyph = font8x8_basic[ch & 0x7fu][glyph_row & 7];
			bool on = (glyph & (1u << (CHAR_WIDTH - 1 - (x % CHAR_WIDTH)))) != 0;
			if (on) {
				color = char_colors[col];
			}
		}
		pixels[x] = color;
	}

	raw_scanline_finish(dest);
}

static void __time_critical_func(render_loop)(void) {
	static uint32_t last_frame_num;
	int core_num = get_core_num();

	printf("siddler_pico renderer started on core %d\n", core_num);

	while (true) {
		struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);

		mutex_enter_blocking(&frame_logic_mutex);
		uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);
		if (frame_num != last_frame_num) {
			last_frame_num = frame_num;
			frame_update_logic();
		}
		mutex_exit(&frame_logic_mutex);

		render_scanline(scanline_buffer, core_num);

		scanvideo_end_scanline_generation(scanline_buffer);
	}
}

int main(void) {
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
	for (int i = 0; i < 500; ++i) {
		tud_task();
		if (tud_mounted()) {
			break;
		}
		sleep_ms(1);
	}
	mutex_init(&frame_logic_mutex);
	critical_section_init(&sid_state_lock);
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, led_state);
	clear_status_lines();
	audio_ready = siddler_audio_init();
    set_status_line(0, "Oddvolt's SIDDLER PICO ALPHA10");
	set_status_line(1, "WAITING FOR SID DATA");
	set_status_line(2, "EVENTS --- AVG --- CHIP -");
	update_cdc_status_line("CDC WAITING");
	set_status_line(4, "BUFFER    0 CDC OFF");
    set_status_line(5, "FRAME 00000000 50Hz|NoMore KRRZPRRTs");
	update_event_trace_display(0, 0);

	scanvideo_setup(&VGA_MODE);
	scanvideo_timing_enable(true);

	multicore_launch_core1(render_loop);

	while (true) {
		process_serial_stream();
		if (audio_ready) {
			siddler_audio_task();
		}
		tight_loop_contents();
	}
	return 0;
}
