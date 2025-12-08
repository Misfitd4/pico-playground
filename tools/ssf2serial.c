#define _POSIX_C_SOURCE 200809L
#define _DARWIN_C_SOURCE 1

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef SID_CLOCK_HZ_DOUBLE
#define SID_CLOCK_HZ_DOUBLE 985248.0
#endif

#ifndef SID_FRAME_CYCLES
#define SID_FRAME_CYCLES 19656.0
#endif

#define DUMP_EVENT_SIZE 4u
#define FRAME_BUFFER_MAX_BYTES (64 * 1024)
#define BLOCK_FRAMES 200
#define OVERLAPFRAMES 100
#define SID_DELAY_ADDR 0xFFu

static volatile sig_atomic_t g_running = 1;
extern char **environ;

static void handle_signal(int sig)
{
  (void)sig;
  g_running = 0;
}

static inline uint32_t decode_delta_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static double cycles_to_us(double cycles)
{
  return (cycles / SID_CLOCK_HZ_DOUBLE) * 1e6;
}

static void sleep_us(double us)
{
  if (us <= 0) {
    return;
  }
  double secs = us / 1e6;
  struct timespec ts;
  ts.tv_sec = (time_t)secs;
  ts.tv_nsec = (long)((secs - (double)ts.tv_sec) * 1e9);
  if (ts.tv_nsec < 0) {
    ts.tv_nsec = 0;
  }
  while (nanosleep(&ts, &ts) < 0 && errno == EINTR) {
    if (!g_running) break;
  }
}

static int wait_for_ready(int serial_fd)
{
  char line[256];
  size_t line_len = 0;
  struct pollfd p = {
    .fd = serial_fd,
    .events = POLLIN,
    .revents = 0,
  };

  while (g_running) {
    int pr = poll(&p, 1, -1);
    if (pr < 0) {
      if (errno == EINTR) continue;
      perror("ssf2serial: poll ready");
      return -1;
    }
    if (!(p.revents & POLLIN)) {
      continue;
    }
    uint8_t ch;
    ssize_t r = read(serial_fd, &ch, 1);
    if (r <= 0) {
      fprintf(stderr, "ssf2serial: device closed while waiting for READY\n");
      return -1;
    }
    write(STDOUT_FILENO, &ch, 1);
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[line_len] = '\0';
      if (strstr(line, "READY")) {
        return 0;
      }
      line_len = 0;
    } else if (line_len + 1 < sizeof line) {
      line[line_len++] = (char)ch;
    } else {
      line_len = 0;
    }
  }
  return -1;
}

static void drain_serial_nonblocking(int serial_fd)
{
  struct pollfd p = {
    .fd = serial_fd,
    .events = POLLIN,
    .revents = 0,
  };
  uint8_t buf[256];
  while (poll(&p, 1, 0) > 0) {
    if (!(p.revents & POLLIN)) break;
    ssize_t r = read(serial_fd, buf, sizeof buf);
    if (r <= 0) break;
    write(STDOUT_FILENO, buf, (size_t)r);
    p.revents = 0;
  }
}

static ssize_t load_frame(int bin_fd, uint8_t *frame_buf, size_t buf_cap,
                          double *out_cycles)
{
  *out_cycles = 0.0;
  size_t frame_len = 0;
  uint8_t event[DUMP_EVENT_SIZE];
  while (*out_cycles < SID_FRAME_CYCLES && g_running) {
    ssize_t r = read(bin_fd, event, DUMP_EVENT_SIZE);
    if (r == 0) {
      break;
    }
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      perror("ssf2serial: read frame");
      return -1;
    }
    if ((size_t)r != DUMP_EVENT_SIZE) {
      fprintf(stderr, "ssf2serial: truncated event in bin\n");
      return -1;
    }
    if (frame_len + DUMP_EVENT_SIZE > buf_cap) {
      fprintf(stderr, "ssf2serial: frame buffer overflow\n");
      return -1;
    }
    memcpy(frame_buf + frame_len, event, DUMP_EVENT_SIZE);
    frame_len += DUMP_EVENT_SIZE;
    uint32_t delta = decode_delta_le(event);
    *out_cycles += (double)delta;
  }
  return (ssize_t)frame_len;
}

static int send_frame(int serial_fd, const uint8_t *frame_buf, size_t frame_len)
{
  size_t written = 0;
  while (written < frame_len && g_running) {
    ssize_t w = write(serial_fd, frame_buf + written, frame_len - written);
    if (w < 0) {
      if (errno == EINTR) continue;
      perror("ssf2serial: write frame");
      return -1;
    }
    written += (size_t)w;
  }
  return 0;
}

/* Stream one .bin file to the Pico, with timing similar to sid2serial */
static int interactive_stream_bin(int serial_fd, const char *bin_path, bool wait_for_ready_flag)
{
  int bin_fd = open(bin_path, O_RDONLY);
  if (bin_fd < 0) {
    perror("ssf2serial: open bin");
    return -1;
  }

  if (wait_for_ready_flag) {
    fprintf(stderr, "[ssf] waiting for READY from device...\n");
    if (wait_for_ready(serial_fd) != 0) {
      close(bin_fd);
      return -1;
    }
    fprintf(stderr, "[ssf] device READY, streaming frames\n");
  } else {
    fprintf(stderr, "[ssf] streaming additional hashid from %s\n", bin_path);
  }

  uint8_t frame_buf[FRAME_BUFFER_MAX_BYTES];
  double frame_cycles = 0.0;
  uint64_t total_bytes = 0;
  uint64_t block_bytes = 0;
  uint32_t frames_sent = 0;
  uint32_t frames_in_block = 0;
  double block_sleep_us = 0.0;

  while (g_running) {
    ssize_t frame_len = load_frame(bin_fd, frame_buf, sizeof frame_buf,
                                   &frame_cycles);
    if (frame_len < 0) {
      break;
    }
    if (frame_len == 0) {
      fprintf(stderr,
              "[ssf] stream complete for %s (%llu bytes, %u frames)\n",
              bin_path,
              (unsigned long long)total_bytes,
              frames_sent);
      close(bin_fd);
      return 0;
    }

    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    double frame_start = (double)ts_start.tv_sec * 1e6 +
      (double)ts_start.tv_nsec / 1e3;

    if (send_frame(serial_fd, frame_buf, (size_t)frame_len) != 0) {
      break;
    }

    struct timespec ts_end;
    clock_gettime(CLOCK_MONOTONIC, &ts_end);
    double frame_end = (double)ts_end.tv_sec * 1e6 +
      (double)ts_end.tv_nsec / 1e3;
    double tx_us = frame_end - frame_start;
    if (tx_us < 0) tx_us = 0;

    total_bytes += (uint64_t)frame_len;
    block_bytes += (uint64_t)frame_len;
    frames_sent++;

    drain_serial_nonblocking(serial_fd);

    double target_us = cycles_to_us(frame_cycles) - tx_us;
    if (target_us < 0) target_us = 0;
    block_sleep_us += target_us;
    frames_in_block++;

    fprintf(stderr,
            "[chunk %u] bytes=%zu tx=%.6fms sleep=%.6fms block=%u/%u\n",
            frames_sent,
            (size_t)frame_len,
            tx_us / 1000.0,
            target_us / 1000.0,
            frames_in_block,
            (unsigned)BLOCK_FRAMES);

    if (frames_in_block >= BLOCK_FRAMES) {
      double overlap = cycles_to_us(SID_FRAME_CYCLES * OVERLAPFRAMES);
      double sleep_us_total = block_sleep_us - overlap;
      if (sleep_us_total < 0) sleep_us_total = 0;

      double avg_bytes = frames_in_block ?
        ((double)block_bytes / frames_in_block) : 0.0;

      fprintf(stderr,
              "[block] frames=%u bytes_this_block=%llu total_bytes=%llu "
              "avg=%.2f sleep=%.6fms (overlap %.8fms)\n",
              (unsigned)frames_in_block,
              (unsigned long long)block_bytes,
              (unsigned long long)total_bytes,
              avg_bytes,
              sleep_us_total / 1000.0,
              overlap / 1000.0);

      if (sleep_us_total > 0) {
        sleep_us(sleep_us_total);
        drain_serial_nonblocking(serial_fd);
      }

      block_sleep_us = 0.0;
      frames_in_block = 0;
      block_bytes = 0;
    }
  }

  close(bin_fd);
  return -1;
}

static int open_serial_device(const char *path, long baud)
{
  int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("ssf2serial: open serial");
    return -1;
  }
  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) {
    perror("ssf2serial: tcgetattr");
    close(fd);
    return -1;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  speed_t rate = B115200;
#ifdef B2000000
  if (baud == 2000000) rate = B2000000;
#endif
#ifdef B1500000
  if (baud == 1500000) rate = B1500000;
#endif
#ifdef B1000000
  if (baud == 1000000) rate = B1000000;
#endif
#ifdef B921600
  if (baud == 921600) rate = B921600;
#endif
#ifdef B460800
  if (baud == 460800) rate = B460800;
#endif
#ifdef B230400
  if (baud == 230400) rate = B230400;
#endif
  if (baud == 115200) rate = B115200;
  else if (baud == 57600) rate = B57600;
  else if (baud == 38400) rate = B38400;
  else if (baud == 19200) rate = B19200;
  else if (baud == 9600) rate = B9600;
  cfsetispeed(&tio, rate);
  cfsetospeed(&tio, rate);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    perror("ssf2serial: configure serial");
    close(fd);
    return -1;
  }
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
  }
  return fd;
}

/* ----- Simple SSF parsing and conversion to SID-style .bin ----- */

typedef struct {
  int64_t hashid;
  int64_t last_cycle;
  int     have_last;
} hash_state_t;

/* Emit one 4-byte event: delta (16-bit LE), addr, value */
static int emit_event(FILE *out, uint16_t delta, uint8_t addr, uint8_t value)
{
  uint8_t buf[4];
  buf[0] = (uint8_t)(delta & 0xFF);
  buf[1] = (uint8_t)((delta >> 8) & 0xFF);
  buf[2] = addr;
  buf[3] = value;
  if (fwrite(buf, 1, sizeof buf, out) != sizeof buf) {
    perror("ssf2serial: write bin");
    return -1;
  }
  return 0;
}

/* Emit a pure delay (split into 0xFFFF chunks if needed) */
static int emit_delay(FILE *out, uint32_t delta)
{
  while (delta > 0xFFFFu) {
    if (emit_event(out, 0xFFFFu, SID_DELAY_ADDR, 0) != 0) {
      return -1;
    }
    delta -= 0xFFFFu;
  }
  if (delta > 0) {
    if (emit_event(out, (uint16_t)delta, SID_DELAY_ADDR, 0) != 0) {
      return -1;
    }
  }
  return 0;
}

/*
 * Very rough mapping from SSF columns to SID voice registers.
 * SSF line (CSV) example:
 *   hash, chip, cycle, 0, freq, pw, gate, ..., ..., ..., ..., ..., ..., ..., 19511, hash2, ..., ...
 *
 * We use:
 *   col 0: hashid (int64)
 *   col 1: chip index (0..2)
 *   col 2: absolute cycle
 *   col 4: freq (int, 0..65535)
 *   col 5: pw   (int, 0..4095)
 *   col 6: gate (0/1)
 */
static int process_ssf_line_for_hash(FILE *bin_out,
                                     const char *line,
                                     int64_t target_hash,
                                     hash_state_t *state)
{
  /* We need a scratch copy for strtok */
  char buf[1024];
  size_t len = strlen(line);
  if (len >= sizeof buf) {
    /* Too long, skip */
    return 0;
  }
  memcpy(buf, line, len + 1);

  char *saveptr = NULL;
  char *fields[32];
  int field_count = 0;

  char *tok = strtok_r(buf, ",\r\n", &saveptr);
  while (tok && field_count < (int)(sizeof fields / sizeof fields[0])) {
    fields[field_count++] = tok;
    tok = strtok_r(NULL, ",\r\n", &saveptr);
  }
  if (field_count < 3) {
    return 0;
  }

  /* Parse hashid */
  char *endp = NULL;
  int64_t hash = strtoll(fields[0], &endp, 10);
  if (endp == fields[0]) {
    return 0;
  }

  if (hash != target_hash) {
    return 0;
  }

  /* Parse cycle */
  if (!fields[2][0]) {
    return 0;
  }
  int64_t cycle = strtoll(fields[2], &endp, 10);
  if (endp == fields[2]) {
    return 0;
  }

  /* Compute delta cycles */
  uint32_t delta = 0;
  if (state->have_last) {
    if (cycle <= state->last_cycle) {
      delta = 0;
    } else {
      int64_t diff = cycle - state->last_cycle;
      if (diff > 0xFFFFFFFFLL) diff = 0xFFFFFFFFLL;
      delta = (uint32_t)diff;
    }
  } else {
    delta = 0;
    state->have_last = 1;
  }
  state->last_cycle = cycle;

  /* Emit delay first */
  if (emit_delay(bin_out, delta) != 0) {
    return -1;
  }

  /* Now emit some SID register writes from SSF columns (best-effort guess) */
  int chip_index = 0;
  if (field_count > 1 && fields[1][0]) {
    chip_index = (int)strtol(fields[1], NULL, 10);
    if (chip_index < 0) chip_index = 0;
    if (chip_index > 2) chip_index = 2;
  }

  long freq = 0;
  long pw = 0;
  int gate = 0;

  if (field_count > 4 && fields[4][0]) {
    freq = strtol(fields[4], NULL, 10);
    if (freq < 0) freq = 0;
    if (freq > 65535) freq = 65535;
  }
  if (field_count > 5 && fields[5][0]) {
    pw = strtol(fields[5], NULL, 10);
    if (pw < 0) pw = 0;
    if (pw > 4095) pw = 4095;
  }
  if (field_count > 6 && fields[6][0]) {
    gate = (int)strtol(fields[6], NULL, 10);
    if (gate != 0) gate = 1;
  }

  uint8_t freq_lo = (uint8_t)(freq & 0xFF);
  uint8_t freq_hi = (uint8_t)((freq >> 8) & 0xFF);

  uint8_t pw_lo = (uint8_t)(pw & 0xFF);
  uint8_t pw_hi = (uint8_t)((pw >> 8) & 0x0F); /* 12-bit PW */

  /* Very rough control: gate + triangle waveform */
  uint8_t ctrl = (gate ? 0x01 : 0x00) | 0x10;

  /* Dummy ADSR */
  uint8_t attack_decay = 0xF9;       /* A=F, D=9 */
  uint8_t sustain_release = 0xF6;    /* S=F, R=6 */

  int base = chip_index * 7;

  /* Write FREQ_LO/HIGH, PW_LO/HIGH, CTRL, ADSR. All with delta=0 */
  if (emit_event(bin_out, 0, (uint8_t)(base + 0), freq_lo) != 0) return -1;
  if (emit_event(bin_out, 0, (uint8_t)(base + 1), freq_hi) != 0) return -1;
  if (emit_event(bin_out, 0, (uint8_t)(base + 2), pw_lo)   != 0) return -1;
  if (emit_event(bin_out, 0, (uint8_t)(base + 3), pw_hi)   != 0) return -1;
  if (emit_event(bin_out, 0, (uint8_t)(base + 4), ctrl)    != 0) return -1;
  if (emit_event(bin_out, 0, (uint8_t)(base + 5), attack_decay) != 0) return -1;
  if (emit_event(bin_out, 0, (uint8_t)(base + 6), sustain_release) != 0) return -1;

  return 1; /* event(s) generated */
}

/* Convert all lines for a given hashid into a .bin file */
static int build_bin_for_hash(const char *ssf_path,
                              int64_t hashid,
                              const char *bin_path)
{
  FILE *in = fopen(ssf_path, "r");
  if (!in) {
    perror("ssf2serial: open ssf");
    return -1;
  }
  FILE *out = fopen(bin_path, "wb");
  if (!out) {
    perror("ssf2serial: open bin");
    fclose(in);
    return -1;
  }

  char *line = NULL;
  size_t cap = 0;
  ssize_t n;
  uint64_t events = 0;
  hash_state_t st = { .hashid = hashid, .last_cycle = 0, .have_last = 0 };

  while ((n = getline(&line, &cap, in)) != -1 && g_running) {
    int r = process_ssf_line_for_hash(out, line, hashid, &st);
    if (r < 0) {
      free(line);
      fclose(in);
      fclose(out);
      return -1;
    }
    if (r > 0) events += (uint64_t)r;
  }

  free(line);
  fclose(in);
  fclose(out);

  fprintf(stderr, "[ssf] hashid %lld -> %s (%llu events)\n",
          (long long)hashid,
          bin_path,
          (unsigned long long)events);
  return 0;
}

/* Dynamic list for hashids */
typedef struct {
  int64_t *data;
  size_t   count;
  size_t   cap;
} hash_list_t;

/* Collect all unique hashids from the SSF file (dynamic grow) */
static int collect_hashids(const char *ssf_path,
                           hash_list_t *list)
{
  list->data  = NULL;
  list->count = 0;
  list->cap   = 0;

  FILE *in = fopen(ssf_path, "r");
  if (!in) {
    perror("ssf2serial: open ssf");
    return -1;
  }

  char *line = NULL;
  size_t cap = 0;
  ssize_t n;

  while ((n = getline(&line, &cap, in)) != -1 && g_running) {
    if (n <= 0) continue;
    char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (!*p || *p == '#') continue;

    char *endp = NULL;
    int64_t hash = strtoll(p, &endp, 10);
    if (endp == p) continue;

    /* check if already present */
    bool found = false;
    for (size_t i = 0; i < list->count; ++i) {
      if (list->data[i] == hash) {
        found = true;
        break;
      }
    }
    if (!found) {
      if (list->count == list->cap) {
        size_t new_cap = list->cap ? list->cap * 2 : 128;
        int64_t *new_data = realloc(list->data, new_cap * sizeof(int64_t));
        if (!new_data) {
          perror("ssf2serial: realloc hash list");
          free(line);
          fclose(in);
          return -1;
        }
        list->data = new_data;
        list->cap  = new_cap;
      }
      list->data[list->count++] = hash;
    }
  }

  free(line);
  fclose(in);
  fprintf(stderr, "[ssf] found %zu unique hashids\n", list->count);
  return 0;
}

static void usage(const char *prog)
{
  fprintf(stderr,
          "Usage: %s -f <serial> [-b <baud>] [-h <hashid>]\n"
          "\n"
          "Reads SSF CSV from stdin (e.g. via zstdcat) and streams\n"
          "SID-style events to the Pico. If -h is omitted, all\n"
          "hashids are played in sequence with 1s delay between.\n"
          "\n"
          "Example (single hash):\n"
          "  zstdcat tune.ssf.zst | %s -h -8316251235258051595 \\\n"
          "      -f /dev/cu.usbmodem00011 -b 2000000\n"
          "\n"
          "Example (all hashids):\n"
          "  zstdcat tune.ssf.zst | %s -f /dev/cu.usbmodem00011 -b 2000000\n",
          prog ? prog : "ssf2serial",
          prog ? prog : "ssf2serial",
          prog ? prog : "ssf2serial");
}

int main(int argc, char **argv)
{
  const char *serial_dev = NULL;
  long baud = 2000000;
  int64_t target_hashid = 0;
  int have_hashid = 0;

  int opt;
  while ((opt = getopt(argc, argv, "f:b:h:")) != -1) {
    switch (opt) {
      case 'f':
        serial_dev = optarg;
        break;
      case 'b':
        baud = strtol(optarg, NULL, 10);
        if (baud <= 0) {
          fprintf(stderr, "Invalid baud '%s'\n", optarg);
          return 1;
        }
        break;
      case 'h':
        target_hashid = strtoll(optarg, NULL, 10);
        have_hashid = 1;
        break;
      default:
        usage(argv[0]);
        return 1;
    }
  }

  if (!serial_dev) {
    usage(argv[0]);
    return 1;
  }

  /* Workdir for temporary files */
  char workdir_template[] = "/tmp/ssf2serial-XXXXXX";
  char *workdir = mkdtemp(workdir_template);
  if (!workdir) {
    perror("ssf2serial: mkdtemp");
    return 1;
  }

  /* Install signal handlers */
  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_handler = handle_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  /* Save stdin to an SSF temp file so we can re-read it */
  char ssf_path[PATH_MAX];
  snprintf(ssf_path, sizeof ssf_path, "%s/input.ssf", workdir);

  FILE *ssf_out = fopen(ssf_path, "wb");
  if (!ssf_out) {
    perror("ssf2serial: open ssf temp");
    return 1;
  }

  uint8_t buf[4096];
  ssize_t n;
  while ((n = read(STDIN_FILENO, buf, sizeof buf)) > 0) {
    if (fwrite(buf, 1, (size_t)n, ssf_out) != (size_t)n) {
      perror("ssf2serial: write ssf temp");
      fclose(ssf_out);
      return 1;
    }
  }
  fclose(ssf_out);

  /* Open serial */
  int serial_fd = open_serial_device(serial_dev, baud);
  if (serial_fd < 0) {
    return 1;
  }

  int rc = 0;

  if (have_hashid) {
    /* Single-hash mode */
    char bin_path[PATH_MAX];
    snprintf(bin_path, sizeof bin_path, "%s/hash_%lld.bin",
             workdir, (long long)target_hashid);

    if (build_bin_for_hash(ssf_path, target_hashid, bin_path) != 0) {
      rc = 1;
      goto cleanup;
    }

    if (interactive_stream_bin(serial_fd, bin_path, true) != 0) {
      rc = 1;
      goto cleanup;
    }
  } else {
    /* Multi-hash mode: collect all hashids and play them in sequence */
    hash_list_t list;
    if (collect_hashids(ssf_path, &list) != 0) {
      rc = 1;
      goto cleanup;
    }
    if (!list.count) {
      fprintf(stderr, "ssf2serial: no hashids found in input\n");
      free(list.data);
      rc = 1;
      goto cleanup;
    }

    for (size_t i = 0; i < list.count && g_running; ++i) {
      int64_t h = list.data[i];
      char bin_path[PATH_MAX];
      snprintf(bin_path, sizeof bin_path, "%s/hash_%zu.bin", workdir, i);

      if (build_bin_for_hash(ssf_path, h, bin_path) != 0) {
        rc = 1;
        free(list.data);
        goto cleanup;
      }

      bool wait_ready_flag = (i == 0);
      if (interactive_stream_bin(serial_fd, bin_path, wait_ready_flag) != 0) {
        rc = 1;
        free(list.data);
        goto cleanup;
      }

      if (i + 1 < list.count) {
        fprintf(stderr, "[ssf] waiting 1 second before next hashid...\n");
        usleep(300000);   // 100 ms        
        //  sleep(1);
      }
    }
    free(list.data);
  }

cleanup:
  if (serial_fd >= 0) {
    close(serial_fd);
  }
  /* Clean up temp directory */
  char cmd[PATH_MAX * 2];
  snprintf(cmd, sizeof cmd, "rm -rf '%s'", workdir);
  (void)system(cmd);

  return rc;
}
