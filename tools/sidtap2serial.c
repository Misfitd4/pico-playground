#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <termios.h>
#include <unistd.h>

#define DEFAULT_FIFO "/tmp/sid.tap"
#define SID_MAGIC 0x53494446u

#ifndef B2000000
#define B2000000 B115200
#endif

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t count;
    uint32_t frame;
} sid_header_t;

typedef struct {
    uint8_t addr;
    uint8_t value;
    uint32_t delta;
} sid_event_t;
#pragma pack(pop)

typedef struct {
    uint8_t opcode;
    uint8_t param0;
    uint8_t param1;
    uint8_t param2;
} sid_command_t;

static volatile sig_atomic_t g_running = 1;

typedef struct {
    unsigned long long frames;
    unsigned long long events;
    unsigned long long fifo_bytes;
    unsigned long long serial_bytes;
} sidtap_stats_t;

static sidtap_stats_t g_stats = {0, 0, 0, 0};
static struct timeval g_start_time = {0, 0};
static unsigned long long g_fifo_read_ops = 0;
static uint8_t g_voice_mute_mask = 0;
static bool g_filter_enabled = true;
static int g_keyboard_enabled = 0;
static int g_serial_fd = -1;

#define PAL_CYCLES_PER_FRAME 19656u
#define PAL_FRAME_TOLERANCE 512u
#define PAL_MAX_CYCLE_CARRY (PAL_CYCLES_PER_FRAME * 8u)
#define PAL_EVENT_DELTA_SANITY (PAL_CYCLES_PER_FRAME * 256u)
#define PAL_OVERFLOW_WARN_THRESHOLD 300u
#define SID_FILTER_ADDR_MIN 0x15u
#define SID_FILTER_ADDR_MAX 0x18u
#define SID_CMD_FRAME_COUNT 0xFFFFu
#define SID_CMD_CYCLE_MODE 0x01u
#define SID_CMD_SET_VOICE_MASK 0x02u
#define SID_CMD_SET_FILTER 0x03u

/* Mirror constants from firmware for screen layout */
#define SIDDLER_STATUS_SCREEN_COUNT 4
#define SIDDLER_TEXT_COLS 40
#define SIDDLER_TEXT_ROWS 27

static inline uint32_t clamp_u32(uint64_t value, uint32_t max) {
    return (value > max) ? max : (uint32_t)value;
}

static bool should_drop_sid_event(uint8_t addr);
static void send_sid_command(uint8_t opcode, uint8_t param0, uint8_t param1, uint8_t param2);

static uint16_t le16_to_host(uint16_t v) {
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    return __builtin_bswap16(v);
#else
    return v;
#endif
}

static uint32_t le32_to_host(uint32_t v) {
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    return __builtin_bswap32(v);
#else
    return v;
#endif
}

static uint32_t host_to_le32(uint32_t v) {
    return le32_to_host(v);
}

static uint32_t apply_voice_filtering(sid_event_t *events, uint32_t count) {
    if (!events || count == 0) {
        return 0;
    }
    if (g_voice_mute_mask == 0 && g_filter_enabled) {
        return count;
    }
    uint32_t out = 0;
    uint64_t pending = 0;
    for (uint32_t i = 0; i < count; ++i) {
        uint64_t delta_acc = (uint64_t)events[i].delta + pending;
        if (delta_acc > UINT32_MAX) {
            delta_acc = UINT32_MAX;
        }
        uint32_t adjusted_delta = (uint32_t)delta_acc;
        if (should_drop_sid_event(events[i].addr)) {
            pending = adjusted_delta;
            continue;
        }
        sid_event_t ev = events[i];
        ev.delta = adjusted_delta;
        events[out++] = ev;
        pending = 0;
    }
    return out;
}

static int sid_voice_index_from_addr(uint8_t addr) {
    if (addr <= 0x06u) return 0;
    if (addr >= 0x07u && addr <= 0x0Du) return 1;
    if (addr >= 0x0Eu && addr <= 0x14u) return 2;
    return -1;
}

static bool sid_filter_register(uint8_t addr) {
    return addr >= SID_FILTER_ADDR_MIN && addr <= SID_FILTER_ADDR_MAX;
}

static bool should_drop_sid_event(uint8_t addr) {
    uint8_t reg = addr & 0x1Fu;
    int voice = sid_voice_index_from_addr(reg);
    if (voice >= 0 && (g_voice_mute_mask & (1u << voice))) {
        return true;
    }
    if (!g_filter_enabled && sid_filter_register(reg)) {
        return true;
    }
    return false;
}

static char voice_state_char(int voice) {
    return (g_voice_mute_mask & (1u << voice)) ? '-' : (char)('1' + voice);
}

static void handle_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static ssize_t read_exact(int fd, void *buf, size_t len) {
    uint8_t *p = (uint8_t *)buf;
    size_t total = 0;
    while (total < len) {
        ssize_t n = read(fd, p + total, len - total);
        if (n > 0) {
            total += (size_t)n;
            g_stats.fifo_bytes += (size_t)n;
            g_fifo_read_ops++;
            continue;
        }
        if (n == 0) {
            return 0;  // EOF
        }
        if (errno == EINTR) {
            if (!g_running) return -1;
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            usleep(1000);
            if (!g_running) return -1;
            continue;
        }
        return -1;
    }
    return (ssize_t)total;
}

static void print_stats(void) {
    struct timeval end_time;
    gettimeofday(&end_time, NULL);
    double elapsed = 0.0;
    if (g_start_time.tv_sec != 0 || g_start_time.tv_usec != 0) {
        elapsed = (double)(end_time.tv_sec - g_start_time.tv_sec) +
                  (double)(end_time.tv_usec - g_start_time.tv_usec) / 1e6;
        if (elapsed < 1e-6) elapsed = 1e-6;
    }
    fprintf(stderr,
            "[stats] frames=%llu events=%llu fifo_bytes=%llu serial_bytes=%llu\n",
            g_stats.frames, g_stats.events,
            g_stats.fifo_bytes, g_stats.serial_bytes);
    if (elapsed > 0.0) {
        double fifo_rate = g_stats.fifo_bytes / elapsed / 1024.0;
        double serial_rate = g_stats.serial_bytes / elapsed / 1024.0;
        fprintf(stderr,
                "[stats] duration %.2fs fifo %.2f kB/s serial %.2f kB/s\n",
                elapsed, fifo_rate, serial_rate);
    }
}

static void send_sid_command(uint8_t opcode, uint8_t param0, uint8_t param1, uint8_t param2) {
    if (g_serial_fd < 0) {
        return;
    }
    sid_header_t hdr = {
        .magic = SID_MAGIC,
        .count = SID_CMD_FRAME_COUNT,
        .frame = 0
    };
    sid_command_t cmd = {
        .opcode = opcode,
        .param0 = param0,
        .param1 = param1,
        .param2 = param2
    };
    struct iovec iov[2] = {
        {.iov_base = &hdr, .iov_len = sizeof hdr},
        {.iov_base = &cmd, .iov_len = sizeof cmd}
    };
    ssize_t wrote = writev(g_serial_fd, iov, 2);
    if (wrote != (ssize_t)(sizeof hdr + sizeof cmd)) {
        perror("write control");
    }
}

static void send_sid_command(uint8_t opcode, uint8_t param0, uint8_t param1, uint8_t param2);

typedef struct {
    uint64_t frames;
    uint64_t events_total;
    uint64_t bytes_total;
    uint64_t cdc_bytes_total;
    uint64_t frame_time_total_us;
    uint64_t frame_gap_total_us;
    uint64_t parse_time_total_us;
    uint64_t cdc_reads_total;
    uint64_t prev_read_ops;
    uint32_t max_events;
    uint32_t max_bytes;
    uint32_t max_cdc_bytes;
    uint32_t max_frame_us;
    uint32_t max_frame_gap_us;
    uint32_t max_parse_us;
    uint32_t max_cdc_rate_kbps;
    uint32_t buffer_peak;
    uint32_t last_events;
    uint32_t last_bytes;
    uint32_t last_cdc_bytes;
    uint32_t last_frame_us;
    uint32_t last_frame_gap_us;
    uint32_t last_parse_us;
    uint32_t last_cdc_rate_kbps;
    uint32_t last_cdc_reads;
    struct timeval last_frame_end;
    struct timeval last_print;
    int have_last_frame_end;
    int have_last_print;
    size_t last_buffer_now;
    uint32_t last_frame_cycles;
    uint32_t max_frame_cycles;
    uint32_t cycle_overflows;
    uint32_t cycle_underruns;
} monitor_stats_t;

static monitor_stats_t g_monitor = {0};
static int g_stdout_is_tty = 0;
static uint32_t g_cycle_carry = 0;
static uint32_t g_display_screen = 0;
static char g_screen_lines[SIDDLER_STATUS_SCREEN_COUNT][SIDDLER_TEXT_ROWS][SIDDLER_TEXT_COLS + 1];
static uint8_t g_screen_line_valid[SIDDLER_STATUS_SCREEN_COUNT][SIDDLER_TEXT_ROWS];
static char g_serial_line_buffer[512];
static size_t g_serial_line_len = 0;
static int g_print_help_hint = 1;
static struct termios g_stdin_saved_termios;
static int g_stdin_termios_saved = 0;
static int g_force_status_refresh = 0;
static void pump_serial_input(int serial_fd, int show_status);
static void handle_local_input(int show_status);
static void render_screen_display(uint32_t screen);
static void reset_screen_cache(void);
static void restore_terminal(void);

static inline uint32_t time_diff_us(const struct timeval *start, const struct timeval *end) {
    if (!start || !end) return 0;
    long sec = end->tv_sec - start->tv_sec;
    long usec = end->tv_usec - start->tv_usec;
    if (usec < 0) {
        sec -= 1;
        usec += 1000000;
    }
    if (sec < 0) return 0;
    uint64_t total = (uint64_t)sec * 1000000ull + (uint64_t)usec;
    if (total > UINT32_MAX) total = UINT32_MAX;
    return (uint32_t)total;
}

static void reset_screen_cache(void) {
    for (uint32_t s = 0; s < SIDDLER_STATUS_SCREEN_COUNT; ++s) {
        for (int r = 0; r < SIDDLER_TEXT_ROWS; ++r) {
            memset(g_screen_lines[s][r], ' ', SIDDLER_TEXT_COLS);
            g_screen_lines[s][r][SIDDLER_TEXT_COLS] = '\0';
            g_screen_line_valid[s][r] = 0;
        }
    }
}

static void restore_terminal(void) {
    if (g_stdin_termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_stdin_saved_termios);
        g_stdin_termios_saved = 0;
    }
}

static void print_monitor_status(const monitor_stats_t *st, uint32_t frame_no, size_t buffer_now, uint32_t cycle_carry, int forwarding) {
    if (!st) return;
    if (g_stdout_is_tty) {
        fputs("\033[H\033[2J", stdout);
    }
    uint32_t avg_events = st->frames ? (uint32_t)(st->events_total / st->frames) : 0;
    uint32_t avg_bytes = st->frames ? (uint32_t)(st->bytes_total / st->frames) : 0;
    uint32_t avg_cdc_bytes = (forwarding && st->frames) ? (uint32_t)(st->cdc_bytes_total / st->frames) : 0;
    uint32_t avg_frame_us = st->frames ? (uint32_t)(st->frame_time_total_us / st->frames) : 0;
    uint32_t avg_parse_us = st->frames ? (uint32_t)(st->parse_time_total_us / st->frames) : 0;
    uint32_t avg_frame_gap_us = (st->frames > 1) ? (uint32_t)(st->frame_gap_total_us / (st->frames - 1)) : 0;
    uint32_t avg_cdc_rate = (forwarding && st->frame_time_total_us)
                                ? (uint32_t)((st->cdc_bytes_total * 1000ull) / st->frame_time_total_us)
                                : 0;
    printf("=== SIDTap Status ===\n");
    printf("Frame %08u  Total %llu  Buffer %zu  FIFO pk %u\n",
           frame_no,
           (unsigned long long)st->frames,
           buffer_now,
           st->buffer_peak);
    printf("Events L%u A%u M%u | Bytes L%u A%u M%u\n",
           st->last_events, avg_events, st->max_events,
           st->last_bytes, avg_bytes, st->max_bytes);
    printf("Frameus L%u A%u M%u | Gapus L%u A%u M%u\n",
           st->last_frame_us, avg_frame_us, st->max_frame_us,
           st->last_frame_gap_us, avg_frame_gap_us, st->max_frame_gap_us);
    printf("CDC bytes L%u A%u M%u (%s) | Rate L%u A%u M%u | Reads L%u Tot %llu\n",
           st->last_cdc_bytes, avg_cdc_bytes, st->max_cdc_bytes,
           forwarding ? "ON" : "OFF",
           st->last_cdc_rate_kbps, avg_cdc_rate, st->max_cdc_rate_kbps,
           st->last_cdc_reads, (unsigned long long)st->cdc_reads_total);
    printf("Parseus L%u A%u M%u | Cycles L%u M%u Carry %u OF%u UF%u\n",
           st->last_parse_us, avg_parse_us, st->max_parse_us,
           st->last_frame_cycles,
           st->max_frame_cycles,
           cycle_carry,
           st->cycle_overflows,
           st->cycle_underruns);
    printf("Voices %c%c%c  Filter %s\n",
           voice_state_char(0),
           voice_state_char(1),
           voice_state_char(2),
           g_filter_enabled ? "ON" : "OFF");
    if (g_print_help_hint && g_keyboard_enabled) {
        printf("Keys: n/p screen, 1-3 voices, 0 all, F filter, M mode, q quit, h hide hint.\n");
    }
    printf("\n--- Screen %u ---\n", g_display_screen);
    render_screen_display(g_display_screen);
    fflush(stdout);
}

static void render_screen_display(uint32_t screen) {
    if (screen >= SIDDLER_STATUS_SCREEN_COUNT) {
        printf("(invalid screen)\n");
        return;
    }
    int valid_rows = 0;
    for (int row = 0; row < SIDDLER_TEXT_ROWS; ++row) {
        if (g_screen_line_valid[screen][row]) {
            valid_rows++;
        }
    }
    if (valid_rows == 0) {
        printf("(screen %u: no data yet)\n", screen);
        return;
    }
    for (int row = 0; row < SIDDLER_TEXT_ROWS; ++row) {
        if (g_screen_line_valid[screen][row]) {
            printf("%.*s\n", SIDDLER_TEXT_COLS, g_screen_lines[screen][row]);
        } else {
            printf("~\n");
        }
    }
}

static void process_serial_line(const char *line) {
    if (!line) return;
    if (strncmp(line, "#L ", 3) != 0) {
        return;
    }
    const char *p = line + 3;
    char *endptr;
    unsigned long screen = strtoul(p, &endptr, 10);
    if (endptr == p || screen >= SIDDLER_STATUS_SCREEN_COUNT) {
        return;
    }
    while (*endptr == ' ') endptr++;
    long row = strtol(endptr, &endptr, 10);
    if (endptr == NULL || row < 0 || row >= SIDDLER_TEXT_ROWS) {
        return;
    }
    while (*endptr == ' ') endptr++;
    const char *text = endptr;
    size_t len = strlen(text);
    while (len > 0 && (text[len - 1] == '\r' || text[len - 1] == '\n')) {
        len--;
    }
    char new_line[SIDDLER_TEXT_COLS];
    memset(new_line, ' ', SIDDLER_TEXT_COLS);
    if (len > 0) {
        if (len > SIDDLER_TEXT_COLS) len = SIDDLER_TEXT_COLS;
        memcpy(new_line, text, len);
    }
    if (memcmp(g_screen_lines[screen][row], new_line, SIDDLER_TEXT_COLS) != 0) {
        memcpy(g_screen_lines[screen][row], new_line, SIDDLER_TEXT_COLS);
        g_screen_lines[screen][row][SIDDLER_TEXT_COLS] = '\0';
        g_screen_line_valid[screen][row] = 1;
        g_force_status_refresh = 1;
    }
}

static void pump_serial_input(int serial_fd, int show_status) {
    (void) show_status;
    if (serial_fd < 0) return;
    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(serial_fd, &rfds);
        struct timeval tv = {0, 0};
        int rv = select(serial_fd + 1, &rfds, NULL, NULL, &tv);
        if (rv <= 0) {
            break;
        }
        char buf[256];
        ssize_t n = read(serial_fd, buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        for (ssize_t i = 0; i < n; ++i) {
            char ch = buf[i];
            if (ch == '\r') continue;
            if (ch == '\n') {
                g_serial_line_buffer[g_serial_line_len] = '\0';
                process_serial_line(g_serial_line_buffer);
                g_serial_line_len = 0;
            } else {
                if (g_serial_line_len + 1 < sizeof g_serial_line_buffer) {
                    g_serial_line_buffer[g_serial_line_len++] = ch;
                } else {
                    g_serial_line_len = 0;
                }
            }
        }
    }
}

static void handle_local_input(int show_status) {
    if (!g_keyboard_enabled) return;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    struct timeval tv = {0, 0};
    int rv = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    if (rv <= 0) {
        return;
    }
    char ch;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n <= 0) return;
    int refresh = 0;
    switch (ch) {
        case 'n':
        case 'N':
            if (show_status) {
                g_display_screen = (g_display_screen + 1) % SIDDLER_STATUS_SCREEN_COUNT;
                refresh = 1;
            }
            break;
        case 'p':
        case 'P':
            if (show_status) {
                g_display_screen = (g_display_screen + SIDDLER_STATUS_SCREEN_COUNT - 1) % SIDDLER_STATUS_SCREEN_COUNT;
                refresh = 1;
            }
            break;
        case '1':
        case '2':
        case '3': {
            int voice = ch - '1';
            if (voice >= 0 && voice < 3) {
                g_voice_mute_mask ^= (1u << voice);
                fprintf(stderr, "[info] voice %d %s\n", voice + 1,
                        (g_voice_mute_mask & (1u << voice)) ? "muted" : "enabled");
                send_sid_command(SID_CMD_SET_VOICE_MASK, g_voice_mute_mask, 0, 0);
                refresh = show_status;
            }
            break;
        }
        case '0':
            g_voice_mute_mask = 0;
            fprintf(stderr, "[info] all voices enabled\n");
            send_sid_command(SID_CMD_SET_VOICE_MASK, g_voice_mute_mask, 0, 0);
            refresh = show_status;
            break;
        case 'f':
        case 'F':
            g_filter_enabled = !g_filter_enabled;
            fprintf(stderr, "[info] filter registers %s\n",
                    g_filter_enabled ? "enabled" : "muted");
            send_sid_command(SID_CMD_SET_FILTER, g_filter_enabled ? 1u : 0u, 0, 0);
            refresh = show_status;
            break;
        case 'm':
        case 'M':
            fprintf(stderr, "[info] cycling SID model\n");
            send_sid_command(SID_CMD_CYCLE_MODE, 0, 0, 0);
            refresh = show_status;
            break;
        case 'q':
        case 'Q':
            g_running = 0;
            raise(SIGINT);
            return;
        case 'h':
        case 'H':
            if (show_status) {
                g_print_help_hint = 0;
                refresh = 1;
            }
            break;
        default:
            break;
    }
    if (refresh) {
        g_force_status_refresh = 1;
    }
}

static void monitor_update(monitor_stats_t *st,
                           uint32_t frame_no,
                           uint32_t events,
                           uint32_t frame_bytes,
                           uint32_t frame_cycles,
                           uint32_t cycle_carry,
                           const struct timeval *frame_start,
                           int forwarding,
                           int fifo_fd,
                           int show_status) {
    if (!st) return;

    pump_serial_input(g_serial_fd, show_status);
    handle_local_input(show_status);

    struct timeval now;
    gettimeofday(&now, NULL);

    uint32_t frame_us = frame_start ? time_diff_us(frame_start, &now) : 0;
    if (frame_us == 0) frame_us = 1;

    uint32_t frame_gap_us = 0;
    if (st->have_last_frame_end) {
        frame_gap_us = time_diff_us(&st->last_frame_end, frame_start ? frame_start : &now);
    }
    st->last_frame_end = now;
    st->have_last_frame_end = 1;

    st->frames++;
    st->last_events = events;
    st->events_total += events;
    if (events > st->max_events) st->max_events = events;

    st->last_bytes = frame_bytes;
    st->bytes_total += frame_bytes;
    if (frame_bytes > st->max_bytes) st->max_bytes = frame_bytes;

    st->frame_time_total_us += frame_us;
    st->last_frame_us = frame_us;
    if (frame_us > st->max_frame_us) st->max_frame_us = frame_us;

    st->last_parse_us = frame_us;
    st->parse_time_total_us += frame_us;
    if (frame_us > st->max_parse_us) st->max_parse_us = frame_us;

    if (st->frames > 1) {
        st->last_frame_gap_us = frame_gap_us;
        st->frame_gap_total_us += frame_gap_us;
        if (frame_gap_us > st->max_frame_gap_us) st->max_frame_gap_us = frame_gap_us;
    } else {
        st->last_frame_gap_us = 0;
    }

    if (forwarding) {
        st->last_cdc_bytes = frame_bytes;
        st->cdc_bytes_total += frame_bytes;
        if (frame_bytes > st->max_cdc_bytes) st->max_cdc_bytes = frame_bytes;
        uint32_t rate = (uint32_t)((uint64_t)frame_bytes * 1000ull / frame_us);
        st->last_cdc_rate_kbps = rate;
        if (rate > st->max_cdc_rate_kbps) st->max_cdc_rate_kbps = rate;
    } else {
        st->last_cdc_bytes = 0;
        st->last_cdc_rate_kbps = 0;
    }

    st->last_frame_cycles = frame_cycles;
    if (frame_cycles > st->max_frame_cycles) st->max_frame_cycles = frame_cycles;
    if (frame_cycles > PAL_CYCLES_PER_FRAME + PAL_FRAME_TOLERANCE) {
        st->cycle_overflows++;
    }
    if (frame_cycles + cycle_carry + PAL_FRAME_TOLERANCE < PAL_CYCLES_PER_FRAME) {
        st->cycle_underruns++;
    }

    uint64_t reads_now = g_fifo_read_ops;
    uint32_t reads_frame = (uint32_t)(reads_now - st->prev_read_ops);
    st->prev_read_ops = reads_now;
    st->last_cdc_reads = reads_frame;
    st->cdc_reads_total += reads_frame;

    size_t buffer_now = 0;
    if (fifo_fd >= 0) {
        int bytes_available = 0;
        if (ioctl(fifo_fd, FIONREAD, &bytes_available) == 0 && bytes_available > 0) {
            buffer_now = (size_t)bytes_available;
            if (buffer_now > st->buffer_peak) {
                st->buffer_peak = (uint32_t)buffer_now;
            }
        }
    }
    st->last_buffer_now = buffer_now;

    if (!show_status) {
        return;
    }

    int should_print = 0;
    if (!st->have_last_print) {
        should_print = 1;
    } else {
        uint32_t since_last = time_diff_us(&st->last_print, &now);
        if (since_last >= 250000) { // 250 ms
            should_print = 1;
        }
    }

    if (g_force_status_refresh) {
        should_print = 1;
        g_force_status_refresh = 0;
    }

    if (should_print) {
        print_monitor_status(st, frame_no, buffer_now, cycle_carry, forwarding);
        st->last_print = now;
        st->have_last_print = 1;
    }
}
static int open_fifo_nonblock(const char *path) {
    int fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd >= 0) return fd;
    if (errno == ENOENT) {
        if (mkfifo(path, 0666) == 0 || errno == EEXIST) {
            fd = open(path, O_RDONLY);
        }
    }
    return fd;
}

static int open_fifo_blocking(const char *path) {
    int fd;
    do {
        fd = open(path, O_RDONLY);
    } while (fd < 0 && errno == EINTR && g_running);
    return fd;
}


static int open_serial(const char *path, speed_t baud) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    int oflags = fcntl(fd, F_GETFL, 0);
    if (oflags >= 0) {
        fcntl(fd, F_SETFL, oflags & ~O_NONBLOCK);
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) == 0) {
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        cfsetispeed(&tio, baud);
        cfsetospeed(&tio, baud);
        tcsetattr(fd, TCSANOW, &tio);
    }

#ifdef TIOCMBIS
    int flags = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &flags);
#endif
#ifdef TIOCSDTR
    ioctl(fd, TIOCSDTR);
#endif
    return fd;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [-i <fifo>] [-f <serial_dev>] [-b <baud>] [-v|-vv|-vvv]\n"
            "  -i <fifo>  Input FIFO (default %s)\n"
            "  -f <dev>   Forward events to serial CDC device\n"
            "  -b <baud>  Serial baud (default 2000000, ignored for USB CDC)\n"
            "  -v         Verbose (frame summaries)\n"
            "  -vv        Very verbose (events)\n"
            "  -vvv       Event dump plus hexdump\n"
            "  -s         Live status view (n/p switch screen, 1-3/0 voices, F filter, M mode, q quit)\n",
            prog, DEFAULT_FIFO);
}

int main(int argc, char **argv) {
    const char *fifo_path = DEFAULT_FIFO;
    const char *serial_path = NULL;
    int verbose = 0;
    int show_status = 0;
    speed_t baud = B2000000;

    int opt;
    while ((opt = getopt(argc, argv, "i:f:b:vhs")) != -1) {
        switch (opt) {
            case 'i':
                fifo_path = optarg;
                break;
            case 'f':
                serial_path = optarg;
                break;
            case 'b': {
                long val = strtol(optarg, NULL, 10);
                baud = (val >= 2000000) ? B2000000 : B115200;
                break;
            }
            case 'v':
                verbose++;
                break;
            case 's':
                show_status = 1;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }

    reset_screen_cache();

    if (show_status) {
        g_stdout_is_tty = isatty(STDOUT_FILENO);
        g_force_status_refresh = 1;
        setvbuf(stdout, NULL, _IONBF, 0);
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    gettimeofday(&g_start_time, NULL);
    atexit(print_stats);

    int stdin_is_tty = isatty(STDIN_FILENO);
    g_keyboard_enabled = stdin_is_tty;
    if (show_status && isatty(STDOUT_FILENO)) {
        g_stdout_is_tty = 1;
        g_force_status_refresh = 1;
        setvbuf(stdout, NULL, _IONBF, 0);
    }

    if (stdin_is_tty && (show_status || g_keyboard_enabled)) {
        if (tcgetattr(STDIN_FILENO, &g_stdin_saved_termios) == 0) {
            struct termios tio = g_stdin_saved_termios;
            tio.c_lflag &= ~(ICANON);
            tio.c_lflag &= ~(ECHO);
            tio.c_lflag |= ISIG;
#ifdef VINTR
            tio.c_cc[VINTR] = 0x03;
#endif
#ifdef VQUIT
            tio.c_cc[VQUIT] = 0x1c;
#endif
            tio.c_cc[VMIN] = 0;
            tio.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) == 0) {
                g_stdin_termios_saved = 1;
                atexit(restore_terminal);
            }
        }
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }
    } else {
        g_keyboard_enabled = 0;
    }

    if (g_keyboard_enabled) {
        fprintf(stderr, "[info] keyboard controls: 1-3 toggle voices, 0 enable all, F filter, M cycle SID mode, q quit.\n");
    }

    int serial_fd = -1;
    if (serial_path) {
        serial_fd = open_serial(serial_path, baud);
        if (serial_fd < 0) {
            perror("open serial");
            return 1;
        }
        fprintf(stderr, "sidtap2serial: streaming %s -> %s\n", fifo_path, serial_path);
    } else {
        fprintf(stderr, "sidtap2serial: watching %s (no forwarding)\n", fifo_path);
    }
    g_serial_fd = serial_fd;

    while (g_running) {
        pump_serial_input(serial_fd, show_status);
        handle_local_input(show_status);

        int fifo_fd = open_fifo_nonblock(fifo_path);
        if (fifo_fd < 0) {
            perror("open fifo");
            sleep(1);
            continue;
        }

        uint8_t flush_buf[256];
        ssize_t drained = 0;
        while ((drained = read(fifo_fd, flush_buf, sizeof(flush_buf))) > 0) {
            if (!g_running) break;
        }
        close(fifo_fd);
        if (!g_running) break;

        if (verbose) {
            fprintf(stderr, "[info] waiting for fifo writer...\n");
        }

        fifo_fd = open_fifo_blocking(fifo_path);
        if (fifo_fd < 0) {
            if (!g_running) break;
            perror("open fifo");
            sleep(1);
            continue;
        }

        memset(&g_monitor, 0, sizeof(g_monitor));
        g_monitor.prev_read_ops = g_fifo_read_ops;
        g_monitor.have_last_frame_end = 0;
        g_monitor.have_last_print = 0;
        g_cycle_carry = 0;
        g_print_help_hint = 1;

        while (g_running) {
            pump_serial_input(serial_fd, show_status);
            handle_local_input(show_status);

            sid_header_t hdr;
            ssize_t n = read_exact(fifo_fd, &hdr, sizeof(hdr));
            if (n == 0) {
                if (verbose) {
                    fprintf(stderr, "[info] fifo writer closed\n");
                }
                close(fifo_fd);
                fifo_fd = -1;
                break;
            }
            if (n < 0) {
                perror("read header");
                break;
            }

            uint32_t magic = le32_to_host(hdr.magic);
            if (magic != SID_MAGIC) {
                if (verbose) {
                    fprintf(stderr, "[warn] bad magic 0x%08x, resync\n", magic);
                }
                continue;
            }

            sid_header_t raw_hdr = hdr;
            uint32_t count = le16_to_host(hdr.count);
            uint32_t frame_no = le32_to_host(hdr.frame);
            struct timeval frame_start;
            gettimeofday(&frame_start, NULL);

            sid_event_t *events = NULL;
            if (count > 0) {
                events = (sid_event_t *)malloc(count * sizeof(*events));
                if (!events) {
                    fprintf(stderr, "[error] malloc failed for %u events\n", count);
                    g_running = 0;
                    break;
                }
            }

            uint32_t processed = 0;
            bool frame_delta_warning = false;
            uint32_t large_delta = 0;
            uint32_t large_delta_index = 0;
            for (; processed < count && g_running; ++processed) {
                sid_event_t raw_ev;
                if (read_exact(fifo_fd, &raw_ev, sizeof(raw_ev)) <= 0) {
                    g_running = 0;
                    break;
                }
                events[processed] = raw_ev;
                events[processed].delta = le32_to_host(raw_ev.delta);
                if (!frame_delta_warning && events[processed].delta > PAL_EVENT_DELTA_SANITY) {
                    frame_delta_warning = true;
                    large_delta = events[processed].delta;
                    large_delta_index = processed;
                }
                if (verbose > 1) {
                    fprintf(stderr, "  addr=$%02x val=$%02x dt=%u%s\n",
                            events[processed].addr & 0x1fu,
                            events[processed].value,
                            events[processed].delta,
                            serial_fd >= 0 ? "" : " (not forwarded)");
                }
            }

            if (!g_running) {
                free(events);
                break;
            }

            if (processed != count) {
                if (verbose) {
                    fprintf(stderr, "[warn] truncated frame #%u (%u/%u events)\n",
                            frame_no, processed, count);
                }
                free(events);
                continue;
            }

            if (frame_delta_warning && verbose) {
                fprintf(stderr,
                        "[warn] frame #%u event %u delta %u exceeds sanity limit %u (continuing)\n",
                        frame_no, large_delta_index, large_delta, PAL_EVENT_DELTA_SANITY);
            }

            uint32_t original_count = count;
            count = apply_voice_filtering(events, count);
            if (verbose > 1 && original_count != count) {
                fprintf(stderr, "[info] frame #%u filtered %u -> %u events\n",
                        frame_no, original_count, count);
            }
            if (verbose) {
                fprintf(stderr, "[frame] #%u events=%u\n", frame_no, count);
            }

            g_stats.frames++;
            g_stats.events += count;

            uint32_t carry = g_cycle_carry;
            g_cycle_carry = 0;
            uint64_t total_cycles = carry;

            if (count > 0 && carry > 0) {
                uint64_t first_delta = (uint64_t)events[0].delta + carry;
                if (first_delta > UINT32_MAX) {
                    uint64_t overflow = first_delta - UINT32_MAX;
                    events[0].delta = UINT32_MAX;
                    g_cycle_carry += (uint32_t)clamp_u32(overflow, PAL_MAX_CYCLE_CARRY);
                } else {
                    events[0].delta = (uint32_t)first_delta;
                }
            }

            for (uint32_t i = 0; i < count; ++i) {
                total_cycles += events[i].delta;
            }

            uint32_t max_cycles = PAL_CYCLES_PER_FRAME + PAL_FRAME_TOLERANCE;
            if (count > 0 && total_cycles > max_cycles) {
                uint64_t overflow = total_cycles - max_cycles;
                uint32_t reduction = (overflow > UINT32_MAX) ? UINT32_MAX : (uint32_t)overflow;
                if (reduction >= events[count - 1].delta) {
                    reduction = (events[count - 1].delta > 0) ? (events[count - 1].delta - 1u) : 0u;
                }
                if (reduction > 0) {
                    events[count - 1].delta -= reduction;
                    total_cycles -= reduction;
                    g_cycle_carry += reduction;
                    if (verbose && reduction >= PAL_OVERFLOW_WARN_THRESHOLD) {
                        fprintf(stderr, "[warn] frame #%u overflow %u cycles, clamped\n",
                                frame_no, reduction);
                    }
                } else {
                    if (overflow > 0 && g_cycle_carry < PAL_MAX_CYCLE_CARRY) {
                        uint32_t extra = (overflow > UINT32_MAX) ? UINT32_MAX : (uint32_t)overflow;
                        g_cycle_carry += extra;
                    }
                }
            }

            if (total_cycles < PAL_CYCLES_PER_FRAME) {
                uint32_t deficit = (uint32_t)(PAL_CYCLES_PER_FRAME - total_cycles);
                g_cycle_carry += deficit;
            }

            if (g_cycle_carry > PAL_MAX_CYCLE_CARRY) {
                g_cycle_carry = PAL_MAX_CYCLE_CARRY;
            }

		if (serial_fd >= 0) {
			size_t total_bytes = sizeof(raw_hdr) + count * sizeof(sid_event_t);
			struct iovec iov[2];
			iov[0].iov_base = &raw_hdr;
			iov[0].iov_len = sizeof(raw_hdr);
			int iov_count = 1;
			if (count > 0) {
				for (uint32_t i = 0; i < count; ++i) {
					events[i].delta = host_to_le32(events[i].delta);
				}
				iov[1].iov_base = events;
				iov[1].iov_len = count * sizeof(sid_event_t);
				iov_count = 2;
			}
			ssize_t wrote = writev(serial_fd, iov, iov_count);
			if (wrote < 0 || (size_t) wrote != total_bytes) {
				perror("write frame");
				free(events);
				g_running = 0;
				break;
			}
			g_stats.serial_bytes += (size_t) wrote;
		}

            uint32_t frame_bytes = (uint32_t)(sizeof(raw_hdr) + count * sizeof(sid_event_t));
            uint32_t frame_cycles = (total_cycles > UINT32_MAX) ? UINT32_MAX : (uint32_t)total_cycles;
            monitor_update(&g_monitor,
                           frame_no,
                           count,
                           frame_bytes,
                           frame_cycles,
                           g_cycle_carry,
                           &frame_start,
                           serial_fd >= 0,
                           fifo_fd,
                           show_status);

            free(events);
        }

        if (fifo_fd >= 0) {
            close(fifo_fd);
        }
    }

    if (serial_fd >= 0) {
        close(serial_fd);
    }
    g_serial_fd = -1;
    return 0;
}
