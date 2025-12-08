#define _POSIX_C_SOURCE 200809L
#define _DARWIN_C_SOURCE 1

#include <errno.h>
#include <fcntl.h>
#include <ftw.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

#ifndef DEFAULT_VSID_PATH
#define DEFAULT_VSID_PATH "tools/vice-3.9/src/vsid"
#endif

#ifndef SID_CLOCK_HZ_DOUBLE
#define SID_CLOCK_HZ_DOUBLE 985248.0
#endif

#ifndef SID_FRAME_CYCLES
#define SID_FRAME_CYCLES 19656.0
#endif

#define DUMP_EVENT_SIZE 4u
#define FRAME_BUFFER_MAX_BYTES (64 * 1024)
#define BLOCK_FRAMES 2
#define OVERLAPFRAMES 1
#define SID_DELAY_ADDR 0xFFu

static volatile sig_atomic_t g_running = 1;
extern char **environ;

/* --- Runtime filter toggles (keys 1–4) --- */
static bool g_v1_enabled     = true;
static bool g_v2_enabled     = true;
static bool g_v3_enabled     = true;
static bool g_global_enabled = true;

/* --- stdin raw mode handling --- */
static struct termios g_stdin_old;
static bool g_stdin_raw = false;

static void handle_signal(int sig)
{
  (void) sig;
  g_running = 0;
}

static inline uint32_t decode_delta_le(const uint8_t *p)
{
  return (uint32_t) p[0] |
    ((uint32_t) p[1] << 8);
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
  ts.tv_sec = (time_t) secs;
  ts.tv_nsec = (long) ((secs - (double) ts.tv_sec) * 1e9);
  if (ts.tv_nsec < 0) {
    ts.tv_nsec = 0;
  }
  while (nanosleep(&ts, &ts) < 0 && errno == EINTR) {
    if (!g_running) break;
  }
}

/* --- stdin raw / restore --- */

static void stdin_set_raw(void)
{
  if (!isatty(STDIN_FILENO)) {
    return;
  }
  struct termios tio;
  if (tcgetattr(STDIN_FILENO, &tio) != 0) {
    perror("sid2serial: tcgetattr(stdin)");
    return;
  }
  g_stdin_old = tio;
  tio.c_lflag &= ~(ICANON | ECHO);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) != 0) {
    perror("sid2serial: tcsetattr(stdin)");
    return;
  }
  g_stdin_raw = true;
}

static void stdin_restore(void)
{
  if (g_stdin_raw) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_stdin_old);
    g_stdin_raw = false;
  }
}

/* --- Filter helpers --- */

/* SID register ranges as offsets 0x00–0x18:
 *  0x00–0x06: voice 1
 *  0x07–0x0D: voice 2
 *  0x0E–0x14: voice 3
 *  0x15–0x18: global/filter/volume
 *
 * Some dumps may include the full $D400-$D41F, so we mask to 0x1F.
 */

static bool is_voice_reg(uint8_t addr, int voice)
{
  uint8_t r = addr & 0x1F;

  switch (voice) {
    case 1:
      return (r >= 0x00 && r <= 0x06);
    case 2:
      return (r >= 0x07 && r <= 0x0D);
    case 3:
      return (r >= 0x0E && r <= 0x14);
    default:
      return false;
  }
}

static bool is_global_reg(uint8_t addr)
{
  uint8_t r = addr & 0x1F;
  return (r >= 0x15 && r <= 0x18);
}

static bool should_filter_addr(uint8_t addr)
{
  if (!g_v1_enabled && is_voice_reg(addr, 1)) {
    return true;
  }
  if (!g_v2_enabled && is_voice_reg(addr, 2)) {
    return true;
  }
  if (!g_v3_enabled && is_voice_reg(addr, 3)) {
    return true;
  }
  if (!g_global_enabled && is_global_reg(addr)) {
    return true;
  }
  return false;
}

static void print_filter_state(void)
{
  fprintf(stderr,
          "[filter] V1=%s V2=%s V3=%s G=%s  (1/2/3/4 to toggle)\n",
          g_v1_enabled ? "ON " : "OFF",
          g_v2_enabled ? "ON " : "OFF",
          g_v3_enabled ? "ON " : "OFF",
          g_global_enabled ? "ON " : "OFF");
}

/* Poll stdin and toggle filters with keys 1–4.
 * 1 -> V1, 2 -> V2, 3 -> V3, 4 -> Global
 * (Optional: you could add 'q' to quit if you want)
 */
static void handle_keyboard_input(void)
{
  if (!g_stdin_raw) {
    return;
  }

  struct pollfd p;
  p.fd = STDIN_FILENO;
  p.events = POLLIN;
  p.revents = 0;

  uint8_t ch;

  while (poll(&p, 1, 0) > 0) {
    if (!(p.revents & POLLIN)) {
      break;
    }
    ssize_t r = read(STDIN_FILENO, &ch, 1);
    if (r <= 0) {
      break;
    }
    bool changed = false;
    switch (ch) {
      case '1':
        g_v1_enabled = !g_v1_enabled;
        changed = true;
        break;
      case '2':
        g_v2_enabled = !g_v2_enabled;
        changed = true;
        break;
      case '3':
        g_v3_enabled = !g_v3_enabled;
        changed = true;
        break;
      case '4':
        g_global_enabled = !g_global_enabled;
        changed = true;
        break;
      /* Uncomment if you want 'q' to stop playback:
      case 'q':
      case 'Q':
        g_running = 0;
        changed = true;
        break;
      */
      default:
        break;
    }
    if (changed) {
      print_filter_state();
    }
    p.revents = 0;
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
      perror("sid2serial: poll ready");
      return -1;
    }
    if (!(p.revents & POLLIN)) {
      continue;
    }
    uint8_t ch;
    ssize_t r = read(serial_fd, &ch, 1);
    if (r <= 0) {
      fprintf(stderr, "sid2serial: device closed while waiting for READY\n");
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
      line[line_len++] = (char) ch;
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
    write(STDOUT_FILENO, buf, (size_t) r);
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
      perror("sid2serial: read frame");
      return -1;
    }
    if ((size_t) r != DUMP_EVENT_SIZE) {
      fprintf(stderr, "sid2serial: truncated event in dump\n");
      return -1;
    }
    if (frame_len + DUMP_EVENT_SIZE > buf_cap) {
      fprintf(stderr, "sid2serial: frame buffer overflow\n");
      return -1;
    }
    memcpy(frame_buf + frame_len, event, DUMP_EVENT_SIZE);
    frame_len += DUMP_EVENT_SIZE;
    uint32_t delta = decode_delta_le(event);
    *out_cycles += (double) delta;
  }
  return (ssize_t) frame_len;
}

/* Send a frame to the device, but apply runtime filters:
 *  - For filtered SID writes, keep the delta bytes (0,1)
 *    and replace addr with SID_DELAY_ADDR and value with 0.
 *  - This preserves timing but mutes selected voices/global regs.
 */
static int send_frame(int serial_fd, const uint8_t *frame_buf, size_t frame_len)
{
  size_t offset = 0;
  while (offset + DUMP_EVENT_SIZE <= frame_len && g_running) {
    uint8_t ev[DUMP_EVENT_SIZE];
    memcpy(ev, frame_buf + offset, DUMP_EVENT_SIZE);

    uint8_t addr = ev[2];
    if (should_filter_addr(addr)) {
      ev[2] = SID_DELAY_ADDR;
      ev[3] = 0;
    }

    size_t written = 0;
    while (written < DUMP_EVENT_SIZE && g_running) {
      ssize_t w = write(serial_fd, ev + written,
                        DUMP_EVENT_SIZE - written);
      if (w < 0) {
        if (errno == EINTR) continue;
        perror("sid2serial: write frame");
        return -1;
      }
      written += (size_t) w;
    }

    offset += DUMP_EVENT_SIZE;
  }
  return 0;
}

static int run_command(char *const argv[])
{
  pid_t pid;
  int rc = posix_spawn(&pid, argv[0], NULL, NULL, argv, environ);
  if (rc != 0) {
    fprintf(stderr, "sid2serial: failed to spawn %s: %s\n",
            argv[0], strerror(rc));
    return -1;
  }
  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    perror("sid2serial: waitpid");
    return -1;
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    fprintf(stderr, "sid2serial: %s exited with status %d\n",
            argv[0], status);
    return -1;
  }
  return 0;
}

static int run_vsid_dump(const char *vsid_path, const char *sid_file,
                         const char *dump_path, const char *limit_ms,
                         const char *tune_str)
{
  char *argv[20];
  int idx = 0;
  argv[idx++] = (char *)vsid_path;
  argv[idx++] = "-console";
  argv[idx++] = "-sounddev";
  argv[idx++] = "dump";
  argv[idx++] = "-soundarg";
  argv[idx++] = (char *)dump_path;
  argv[idx++] = "-warp";
  if (limit_ms && *limit_ms) {
    argv[idx++] = "-limit";
    argv[idx++] = (char *)limit_ms;
  }
  if (tune_str && *tune_str) {
    argv[idx++] = "-tune";
    argv[idx++] = (char *)tune_str;
  }
  argv[idx++] = (char *)sid_file;
  argv[idx] = NULL;
  fprintf(stderr, "[dump] vsid dump -> %s\n", dump_path);
  return run_command(argv);
}

static int convert_dump_to_binary(const char *dump_path, const char *bin_path)
{
  FILE *in = fopen(dump_path, "r");
  if (!in) {
    perror("sid2serial: open dump");
    return -1;
  }
  FILE *out = fopen(bin_path, "wb");
  if (!out) {
    perror("sid2serial: open bin");
    fclose(in);
    return -1;
  }
  char line[256];
  uint64_t line_no = 0;
  uint64_t event_count = 0;
  while (fgets(line, sizeof line, in)) {
    line_no++;
    char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (!*p || *p == '#') {
      continue;
    }
    unsigned long delta, addr, value;
    if (sscanf(p, "%lu %lu %lu", &delta, &addr, &value) != 3) {
      continue;
    }
    uint8_t a = (uint8_t) addr;
    uint8_t v = (uint8_t) value;
    while (delta > 0xFFFFul) {
      uint16_t chunk = 0xFFFFu;
      uint8_t buf_delay[4] = {
        (uint8_t)(chunk & 0xFF),
        (uint8_t)((chunk >> 8) & 0xFF),
        SID_DELAY_ADDR,
        0u
      };
      if (fwrite(buf_delay, 1, sizeof buf_delay, out) != sizeof buf_delay) {
        perror("sid2serial: write bin");
        fclose(in);
        fclose(out);
        return -1;
      }
      event_count++;
      delta -= chunk;
    }
    uint16_t d = (uint16_t) delta;
    uint8_t buf[4] = {
      (uint8_t) (d & 0xFF),
      (uint8_t) ((d >> 8) & 0xFF),
      a,
      v
    };
    if (fwrite(buf, 1, sizeof buf, out) != sizeof buf) {
      perror("sid2serial: write bin");
      fclose(in);
      fclose(out);
      return -1;
    }
    event_count++;
  }
  fclose(in);
  fclose(out);
  fprintf(stderr, "[dump] encoded %llu events\n",
          (unsigned long long) event_count);
  return 0;
}

static int copy_file(const char *src, const char *dst)
{
  int in_fd = open(src, O_RDONLY);
  if (in_fd < 0) {
    perror("sid2serial: open src");
    return -1;
  }
  int out_fd = open(dst, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (out_fd < 0) {
    perror("sid2serial: open dst");
    close(in_fd);
    return -1;
  }
  uint8_t buf[4096];
  ssize_t n;
  while ((n = read(in_fd, buf, sizeof buf)) > 0) {
    ssize_t off = 0;
    while (off < n) {
      ssize_t wrote = write(out_fd, buf + off, (size_t)(n - off));
      if (wrote < 0) {
        perror("sid2serial: copy write");
        close(in_fd);
        close(out_fd);
        return -1;
      }
      off += wrote;
    }
  }
  close(in_fd);
  close(out_fd);
  return (n < 0) ? -1 : 0;
}

static int open_serial_device(const char *path, long baud)
{
  int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("sid2serial: open serial");
    return -1;
  }
  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) {
    perror("sid2serial: tcgetattr");
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
    perror("sid2serial: configure serial");
    close(fd);
    return -1;
  }
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
  }
  return fd;
}

static int unlink_cb(const char *fpath, const struct stat *sb, int typeflag, struct FTW *ftwbuf)
{
  (void) sb;
  (void) ftwbuf;
  switch (typeflag) {
    case FTW_DP:
    case FTW_D:
      return rmdir(fpath);
    default:
      return unlink(fpath);
  }
}

static void remove_tree(const char *path)
{
  if (!path || !*path) return;
  nftw(path, unlink_cb, 10, FTW_DEPTH | FTW_PHYS);
}

static int interactive_stream(int serial_fd, const char *bin_path)
{
  int bin_fd = open(bin_path, O_RDONLY);
  if (bin_fd < 0) {
    perror("sid2serial: open bin");
    return -1;
  }

  fprintf(stderr, "[dump] waiting for READY from device...\n");
  if (wait_for_ready(serial_fd) != 0) {
    close(bin_fd);
    return -1;
  }
  fprintf(stderr, "[dump] device READY, streaming frames\n");
  print_filter_state();

  uint8_t frame_buf[FRAME_BUFFER_MAX_BYTES];
  double frame_cycles = 0.0;
  uint64_t total_bytes = 0;
  uint64_t block_bytes = 0;
  uint32_t frames_sent = 0;
  uint32_t frames_in_block = 0;
  double block_sleep_us = 0.0;

  while (g_running) {
    /* Check keyboard for filter toggles */
    handle_keyboard_input();

    ssize_t frame_len = load_frame(bin_fd, frame_buf, sizeof frame_buf,
                                   &frame_cycles);
    if (frame_len < 0) {
      break;
    }
    if (frame_len == 0) {
      fprintf(stderr, "[dump] stream complete (%llu bytes, %u frames)\n",
              (unsigned long long) total_bytes, frames_sent);
      close(bin_fd);
      return 0;
    }
    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    double frame_start = (double) ts_start.tv_sec * 1e6 +
      (double) ts_start.tv_nsec / 1e3;
    if (send_frame(serial_fd, frame_buf, (size_t) frame_len) != 0) {
      break;
    }
    struct timespec ts_end;
    clock_gettime(CLOCK_MONOTONIC, &ts_end);
    double frame_end = (double) ts_end.tv_sec * 1e6 +
      (double) ts_end.tv_nsec / 1e3;
    double tx_us = frame_end - frame_start;
    if (tx_us < 0) tx_us = 0;
    total_bytes += (uint64_t) frame_len;
    block_bytes += (uint64_t) frame_len;
    frames_sent++;
    drain_serial_nonblocking(serial_fd);
    double target_us = cycles_to_us(frame_cycles) - tx_us;
    if (target_us < 0) target_us = 0;
    block_sleep_us += target_us;
    frames_in_block++;
    fprintf(stderr,
            "[chunk %u] bytes=%zu tx=%.6fms sleep=%.6fms block=%u/%u\n",
            frames_sent,
            (size_t) frame_len,
            tx_us / 1000.0,
            target_us / 1000.0,
            frames_in_block,
            (unsigned) BLOCK_FRAMES);
    if (frames_in_block >= BLOCK_FRAMES) {
      double overlap = cycles_to_us(SID_FRAME_CYCLES * OVERLAPFRAMES);
      double sleep_us_total = block_sleep_us - overlap;
      if (sleep_us_total < 0) sleep_us_total = 0;

      // NEW: average bytes per frame in this block
      double avg_bytes = frames_in_block ? ((double) block_bytes / frames_in_block) : 0.0;

      fprintf(stderr,
              "[block] frames=%u bytes_this_block=%llu total_bytes=%llu "
              "avg=%.2f sleep=%.6fms (overlap %.8fms)\n",
              (unsigned) frames_in_block,
              (unsigned long long) block_bytes,
              (unsigned long long) total_bytes,
              avg_bytes,
              sleep_us_total / 1000.0,
              overlap / 1000.0);

      if (sleep_us_total > 0) {
        sleep_us(sleep_us_total);
        drain_serial_nonblocking(serial_fd);
      }

      block_sleep_us = 0.0;
      frames_in_block = 0;
      block_bytes = 0;   // reset per block
    }	
  }
  close(bin_fd);
  return -1;
}

static void usage(const char *prog)
{
  fprintf(stderr,
          "Usage: %s -i <file.sid> [-f <serial>] [-b <baud>] [-n <tune>]\n"
          "           [-V <vsid>] [-Z <export.bin>] [-l <limit_ms>]\n"
          "During playback: 1/2/3/4 toggle V1/V2/V3/Global filters.\n",
          prog ? prog : "sid2serial");
}

int main(int argc, char **argv)
{
  const char *sid_file = NULL;
  char serial_dev[PATH_MAX] = {0};
  char vsid_path[PATH_MAX];
  char export_path[PATH_MAX] = {0};
  char limit_arg[32] = {0};
  char tune_buf[16] = {0};
  long baud = 2000000;
  int opt;

  strncpy(vsid_path, DEFAULT_VSID_PATH, sizeof vsid_path - 1);

  while ((opt = getopt(argc, argv, "i:f:b:n:V:Z:l:h")) != -1) {
    switch (opt) {
      case 'i':
        sid_file = optarg;
        break;
      case 'f':
        strncpy(serial_dev, optarg, sizeof serial_dev - 1);
        serial_dev[sizeof serial_dev - 1] = '\0';
        break;
      case 'b':
        baud = strtol(optarg, NULL, 10);
        if (baud <= 0) {
          fprintf(stderr, "Invalid baud '%s'\n", optarg);
          return 1;
        }
        break;
      case 'n':
        strncpy(tune_buf, optarg, sizeof tune_buf - 1);
        tune_buf[sizeof tune_buf - 1] = '\0';
        break;
      case 'V':
        strncpy(vsid_path, optarg, sizeof vsid_path - 1);
        vsid_path[sizeof vsid_path - 1] = '\0';
        break;
      case 'Z':
        strncpy(export_path, optarg, sizeof export_path - 1);
        export_path[sizeof export_path - 1] = '\0';
        break;
      case 'l':
        strncpy(limit_arg, optarg, sizeof limit_arg - 1);
        limit_arg[sizeof limit_arg - 1] = '\0';
        break;
      case 'h':
      default:
        usage(argv[0]);
        return (opt == 'h') ? 0 : 1;
    }
  }

  if (!sid_file) {
    usage(argv[0]);
    return 1;
  }

  int serial_fd = -1;
  int rc = 1;

  char workdir_template[] = "/tmp/sid2serial-XXXXXX";
  char *workdir = mkdtemp(workdir_template);
  if (!workdir) {
    perror("sid2serial: mkdtemp");
    return 1;
  }

  char root_name[PATH_MAX];
  const char *base = strrchr(sid_file, '/');
  base = base ? (base + 1) : sid_file;
  strncpy(root_name, base, sizeof root_name - 1);
  root_name[sizeof root_name - 1] = '\0';
  char *dot = strrchr(root_name, '.');
  if (dot) *dot = '\0';

  char dump_path[PATH_MAX];
  char bin_path[PATH_MAX];
  snprintf(dump_path, sizeof dump_path, "%s/%s.dump", workdir, root_name);
  snprintf(bin_path, sizeof bin_path, "%s/%s.bin", workdir, root_name);

  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_handler = handle_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  int vsid_rc = run_vsid_dump(vsid_path, sid_file, dump_path,
                              limit_arg[0] ? limit_arg : NULL,
                              tune_buf[0] ? tune_buf : NULL);
  struct stat st;
  if (stat(dump_path, &st) != 0 || st.st_size == 0) {
    fprintf(stderr, "sid2serial: vsid dump failed\n");
    goto cleanup;
  }
  if (vsid_rc != 0) {
    fprintf(stderr, "sid2serial: vsid exited with %d but dump exists (%lld bytes), continuing\n",
            vsid_rc, (long long) st.st_size);
  }

  if (convert_dump_to_binary(dump_path, bin_path) != 0) {
    goto cleanup;
  }

  if (export_path[0]) {
    if (copy_file(bin_path, export_path) != 0) {
      fprintf(stderr, "sid2serial: failed to export %s\n", export_path);
      goto cleanup;
    }
    fprintf(stderr, "[dump] exported %s\n", export_path);
  }

  if (!serial_dev[0]) {
    rc = 0;
    goto cleanup;
  }

  serial_fd = open_serial_device(serial_dev, baud);
  if (serial_fd < 0) {
    goto cleanup;
  }

  /* Enable keyboard control for filters */
  stdin_set_raw();

  fprintf(stderr, "[dump] session %s -> %s @ %ld baud\n",
          bin_path, serial_dev, baud);
  if (interactive_stream(serial_fd, bin_path) != 0) {
    goto cleanup;
  }

  rc = 0;

cleanup:
  if (serial_fd >= 0) {
    close(serial_fd);
  }
  stdin_restore();
  remove_tree(workdir);
  return rc;
}


