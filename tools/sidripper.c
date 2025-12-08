// tools/ssf2rip.c
// Simple "instrument ripper" for desidulate SSF files.
// Reads CSV from stdin (typically via: zstdcat song.ssf.zst | ssf2rip ...)
// If -hashid is given, prints a Sid Wizard–style header (multispeed + ADSR)
// and a per-frame parameter table for that SSF.
// If -hashid is omitted, just lists all unique hashids and their counts.
//
// Compile:
//   clang -Wall -Wextra -O2 -o tools/ssf2rip tools/ssf2rip.c
//
// Example:
//   zstdcat Bromance-Intro.ssf.zst | tools/ssf2rip
//   zstdcat Bromance-Intro.ssf.zst | tools/ssf2rip -hashid -8316251235258051595

#define _POSIX_C_SOURCE 200809L
#define _DARWIN_C_SOURCE 1

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE   8192
#define MAX_FIELDS 64
#define MAX_HASHIDS 4096

typedef struct {
    long long hashid;
    unsigned long count;
} hashid_entry_t;

typedef struct {
    int idx_hashid;
    int idx_clock;
    int idx_gate1;
    int idx_freq1;
    int idx_pwduty1;
    int idx_pulse1;
    int idx_noise1;
    int idx_tri1;
    int idx_saw1;
    int idx_test1;
    int idx_sync1;
    int idx_ring1;
    int idx_freq3;
    int idx_test3;
    int idx_flt1;
    int idx_fltcoff;
    int idx_fltres;
    int idx_fltlo;
    int idx_fltband;
    int idx_flthi;
    int idx_atk1;
    int idx_dec1;
    int idx_sus1;
    int idx_rel1;
    int idx_vol;
} ssf_columns_t;

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [-hashid <id>]\n"
        "\n"
        "Examples:\n"
        "  zstdcat song.ssf.zst | %s\n"
        "  zstdcat song.ssf.zst | %s -hashid -8316251235258051595\n",
        prog ? prog : "ssf2rip",
        prog ? prog : "ssf2rip",
        prog ? prog : "ssf2rip");
}

static int split_csv(char *line, char **fields, int max_fields) {
    int n = 0;
    char *p = line;
    while (*p && n < max_fields) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

static int parse_header(char *line, ssf_columns_t *cols) {
    char *fields[MAX_FIELDS];
    int n = split_csv(line, fields, MAX_FIELDS);

    // Initialize all indices to -1
    memset(cols, -1, sizeof(*cols));

    for (int i = 0; i < n; ++i) {
        if (!fields[i]) continue;
        if (strcmp(fields[i], "hashid") == 0) cols->idx_hashid = i;
        else if (strcmp(fields[i], "clock") == 0) cols->idx_clock = i;
        else if (strcmp(fields[i], "gate1") == 0) cols->idx_gate1 = i;
        else if (strcmp(fields[i], "freq1") == 0) cols->idx_freq1 = i;
        else if (strcmp(fields[i], "pwduty1") == 0) cols->idx_pwduty1 = i;
        else if (strcmp(fields[i], "pulse1") == 0) cols->idx_pulse1 = i;
        else if (strcmp(fields[i], "noise1") == 0) cols->idx_noise1 = i;
        else if (strcmp(fields[i], "tri1") == 0) cols->idx_tri1 = i;
        else if (strcmp(fields[i], "saw1") == 0) cols->idx_saw1 = i;
        else if (strcmp(fields[i], "test1") == 0) cols->idx_test1 = i;
        else if (strcmp(fields[i], "sync1") == 0) cols->idx_sync1 = i;
        else if (strcmp(fields[i], "ring1") == 0) cols->idx_ring1 = i;
        else if (strcmp(fields[i], "freq3") == 0) cols->idx_freq3 = i;
        else if (strcmp(fields[i], "test3") == 0) cols->idx_test3 = i;
        else if (strcmp(fields[i], "flt1") == 0) cols->idx_flt1 = i;
        else if (strcmp(fields[i], "fltcoff") == 0) cols->idx_fltcoff = i;
        else if (strcmp(fields[i], "fltres") == 0) cols->idx_fltres = i;
        else if (strcmp(fields[i], "fltlo") == 0) cols->idx_fltlo = i;
        else if (strcmp(fields[i], "fltband") == 0) cols->idx_fltband = i;
        else if (strcmp(fields[i], "flthi") == 0) cols->idx_flthi = i;
        else if (strcmp(fields[i], "atk1") == 0) cols->idx_atk1 = i;
        else if (strcmp(fields[i], "dec1") == 0) cols->idx_dec1 = i;
        else if (strcmp(fields[i], "sus1") == 0) cols->idx_sus1 = i;
        else if (strcmp(fields[i], "rel1") == 0) cols->idx_rel1 = i;
        else if (strcmp(fields[i], "vol") == 0) cols->idx_vol = i;
    }

    if (cols->idx_hashid < 0 || cols->idx_clock < 0 || cols->idx_gate1 < 0 ||
        cols->idx_freq1 < 0 || cols->idx_vol < 0) {
        fprintf(stderr,
                "ssf2rip: required columns not found in header\n");
        return -1;
    }
    return 0;
}

static const char *get_field(char **fields, int n, int idx) {
    if (idx < 0 || idx >= n) return "<NA>";
    if (!fields[idx] || fields[idx][0] == '\0') return "<NA>";
    return fields[idx];
}

static long long parse_hashid(const char *s, bool *ok) {
    if (!s || !*s || strcmp(s, "<NA>") == 0) {
        *ok = false;
        return 0;
    }
    char *end = NULL;
    errno = 0;
    long long v = strtoll(s, &end, 10);
    if (errno != 0 || !end || *end != '\0') {
        *ok = false;
        return 0;
    }
    *ok = true;
    return v;
}

int main(int argc, char **argv) {
    const char *prog = (argc > 0 && argv[0]) ? argv[0] : "ssf2rip";
    long long target_hashid = 0;
    bool have_target = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-hashid") == 0) {
            if (i + 1 >= argc) {
                usage(prog);
                return 1;
            }
            char *end = NULL;
            errno = 0;
            target_hashid = strtoll(argv[i + 1], &end, 10);
            if (errno != 0 || !end || *end != '\0') {
                fprintf(stderr, "ssf2rip: invalid hashid '%s'\n", argv[i + 1]);
                return 1;
            }
            have_target = true;
            ++i;
        } else {
            usage(prog);
            return 1;
        }
    }

    char line_buf[MAX_LINE];

    // Read header
    if (!fgets(line_buf, sizeof line_buf, stdin)) {
        fprintf(stderr, "ssf2rip: empty input\n");
        return 1;
    }
    // strip newline
    line_buf[strcspn(line_buf, "\r\n")] = '\0';

    ssf_columns_t cols;
    if (parse_header(line_buf, &cols) != 0) {
        return 1;
    }

    // If no target hashid: collect stats and print at end
    hashid_entry_t hashids[MAX_HASHIDS];
    int hashid_count = 0;

    bool first_row_for_hash = true;
    bool found_any = false;
    unsigned int frame_index = 0;
    int adsr_atk = 0, adsr_dec = 0, adsr_sus = 0, adsr_rel = 0;
    bool adsr_have = false;

    while (fgets(line_buf, sizeof line_buf, stdin)) {
        line_buf[strcspn(line_buf, "\r\n")] = '\0';
        if (line_buf[0] == '\0') continue;

        char *fields[MAX_FIELDS];
        int n = split_csv(line_buf, fields, MAX_FIELDS);
        if (n <= 1) continue;

        const char *hashid_s = get_field(fields, n, cols.idx_hashid);
        bool ok = false;
        long long h = parse_hashid(hashid_s, &ok);
        if (!ok) continue;

        if (!have_target) {
            // collect stats
            int idx = -1;
            for (int i = 0; i < hashid_count; ++i) {
                if (hashids[i].hashid == h) {
                    idx = i;
                    break;
                }
            }
            if (idx < 0) {
                if (hashid_count >= MAX_HASHIDS) {
                    fprintf(stderr,
                            "ssf2rip: too many unique hashids (max %d)\n",
                            MAX_HASHIDS);
                    return 1;
                }
                idx = hashid_count++;
                hashids[idx].hashid = h;
                hashids[idx].count = 0;
            }
            hashids[idx].count++;
            continue;
        }

        if (h != target_hashid) {
            continue;
        }

        // First time we encounter this hashid: print header, ADSR etc.
        if (first_row_for_hash) {
            first_row_for_hash = false;
            found_any = true;

            // Try to grab ADSR from this row
            const char *a_s = get_field(fields, n, cols.idx_atk1);
            const char *d_s = get_field(fields, n, cols.idx_dec1);
            const char *s_s = get_field(fields, n, cols.idx_sus1);
            const char *r_s = get_field(fields, n, cols.idx_rel1);

            if (a_s && d_s && s_s && r_s &&
                strcmp(a_s, "<NA>") != 0 &&
                strcmp(d_s, "<NA>") != 0 &&
                strcmp(s_s, "<NA>") != 0 &&
                strcmp(r_s, "<NA>") != 0) {
                adsr_atk = (int) strtol(a_s, NULL, 10);
                adsr_dec = (int) strtol(d_s, NULL, 10);
                adsr_sus = (int) strtol(s_s, NULL, 10);
                adsr_rel = (int) strtol(r_s, NULL, 10);
                adsr_have = true;
            }

            printf("hashid: %lld\n", target_hashid);
            printf("multispeed: 1\n");
            if (adsr_have) {
                printf("ADSR: %01X%01X%01X%01X\n",
                       adsr_atk & 0xF,
                       adsr_dec & 0xF,
                       adsr_sus & 0xF,
                       adsr_rel & 0xF);
            } else {
                printf("ADSR: ????\n");
            }
            printf("\n");
            printf("frame  clock   gate1  freq1  pwduty1  pulse noise  tri  saw  test sync ring  freq3 test3  flt1 fltcoff fltres fltlo fltband flthi  vol\n");
            printf("-----  ------  -----  -----  -------  ----- ----- ---- ---- ---- ---- ---- ----- ----- ---- ------- ------ ----- ------- ----- ----\n");
        }

        const char *clock_s   = get_field(fields, n, cols.idx_clock);
        const char *gate1_s   = get_field(fields, n, cols.idx_gate1);
        const char *freq1_s   = get_field(fields, n, cols.idx_freq1);
        const char *pwduty1_s = get_field(fields, n, cols.idx_pwduty1);
        const char *pulse1_s  = get_field(fields, n, cols.idx_pulse1);
        const char *noise1_s  = get_field(fields, n, cols.idx_noise1);
        const char *tri1_s    = get_field(fields, n, cols.idx_tri1);
        const char *saw1_s    = get_field(fields, n, cols.idx_saw1);
        const char *test1_s   = get_field(fields, n, cols.idx_test1);
        const char *sync1_s   = get_field(fields, n, cols.idx_sync1);
        const char *ring1_s   = get_field(fields, n, cols.idx_ring1);
        const char *freq3_s   = get_field(fields, n, cols.idx_freq3);
        const char *test3_s   = get_field(fields, n, cols.idx_test3);
        const char *flt1_s    = get_field(fields, n, cols.idx_flt1);
        const char *fltcoff_s = get_field(fields, n, cols.idx_fltcoff);
        const char *fltres_s  = get_field(fields, n, cols.idx_fltres);
        const char *fltlo_s   = get_field(fields, n, cols.idx_fltlo);
        const char *fltband_s = get_field(fields, n, cols.idx_fltband);
        const char *flthi_s   = get_field(fields, n, cols.idx_flthi);
        const char *vol_s     = get_field(fields, n, cols.idx_vol);

        // Keep output texty, similar-ish to the ssf2swi "raw table".
        printf("%5u  %6s  %5s  %5s  %7s  %5s %5s %4s %4s %4s %4s %4s %5s %5s %4s %7s %6s %5s %7s %5s %4s\n",
               frame_index++,
               clock_s,
               gate1_s,
               freq1_s,
               pwduty1_s,
               pulse1_s,
               noise1_s,
               tri1_s,
               saw1_s,
               test1_s,
               sync1_s,
               ring1_s,
               freq3_s,
               test3_s,
               flt1_s,
               fltcoff_s,
               fltres_s,
               fltlo_s,
               fltband_s,
               flthi_s,
               vol_s);
    }

    if (!have_target) {
        // Just listing hashids mode
        if (hashid_count == 0) {
            fprintf(stderr, "ssf2rip: no data rows found\n");
            return 1;
        }
        printf("Found %d unique hashids:\n", hashid_count);
        printf("hashid,count\n");
        for (int i = 0; i < hashid_count; ++i) {
            printf("%lld,%lu\n",
                   hashids[i].hashid,
                   hashids[i].count);
        }
        return 0;
    }

    if (!found_any) {
        fprintf(stderr,
                "ssf2rip: hashid %lld not found in input\n",
                target_hashid);
        return 1;
    }

    return 0;
}
