#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
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
#pragma pack(pop)

static volatile sig_atomic_t g_running = 1;

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

static ssize_t write_exact(int fd, const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    size_t total = 0;
    while (total < len) {
        ssize_t n = write(fd, p + total, len - total);
        if (n > 0) {
            total += (size_t)n;
            continue;
        }
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (!g_running) return -1;
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            struct timeval tv = {0, 1000};
            select(fd + 1, NULL, &wfds, NULL, &tv);
            continue;
        }
        return -1;
    }
    return (ssize_t)total;
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
            "Usage: %s [-i <fifo>] [-f <serial_dev>] [-b <baud>] [-v|-vv]\n"
            "  -i <fifo>  Input FIFO (default %s)\n"
            "  -f <dev>   Forward events to serial CDC device\n"
            "  -b <baud>  Serial baud (default 2000000, ignored for USB CDC)\n"
            "  -v         Verbose (frame summaries)\n"
            "  -vv        Very verbose (log every event)\n",
            prog, DEFAULT_FIFO);
}

int main(int argc, char **argv) {
    const char *fifo_path = DEFAULT_FIFO;
    const char *serial_path = NULL;
    int verbose = 0;
    speed_t baud = B2000000;

    int opt;
    while ((opt = getopt(argc, argv, "i:f:b:vh")) != -1) {
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
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int serial_fd = -1;
    if (serial_path) {
        serial_fd = open_serial(serial_path, baud);
        if (serial_fd < 0) {
            perror("open serial");
            return 1;
        }
        fprintf(stderr, "sidtap2serial: streaming %s -> %s\n", fifo_path, serial_path);
        if (!verbose) verbose = 1;
    } else {
        fprintf(stderr, "sidtap2serial: watching %s (no forwarding)\n", fifo_path);
    }

    while (g_running) {
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

        while (g_running) {
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

            if (hdr.magic != SID_MAGIC) {
                if (verbose) {
                    fprintf(stderr, "[warn] bad magic 0x%08x, resync\n", hdr.magic);
                }
                continue;
            }

            if (verbose) {
                fprintf(stderr, "[frame] #%u events=%u\n", hdr.frame, hdr.count);
            }

            if (serial_fd >= 0) {
                if (write_exact(serial_fd, &hdr, sizeof(hdr)) < 0) {
                    perror("write header");
                    g_running = 0;
                    break;
                }
            }

            for (uint32_t i = 0; i < hdr.count && g_running; ++i) {
                sid_event_t ev;
                if (read_exact(fifo_fd, &ev, sizeof(ev)) <= 0) {
                    g_running = 0;
                    break;
                }
                if (serial_fd >= 0) {
                    if (write_exact(serial_fd, &ev, sizeof(ev)) < 0) {
                        perror("write event");
                        g_running = 0;
                        break;
                    }
                }
                if (verbose > 1) {
                    fprintf(stderr, "  chip=%u addr=$%02x val=$%02x dt=%u%s\n",
                            ev.chip & 1u, ev.addr & 0x1fu, ev.value, ev.delta,
                            serial_fd >= 0 ? "" : " (not forwarded)");
                }
            }
        }

        if (fifo_fd >= 0) {
            close(fifo_fd);
        }
    }

    if (serial_fd >= 0) {
        close(serial_fd);
    }
    return 0;
}
