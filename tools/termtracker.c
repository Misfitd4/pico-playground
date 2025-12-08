// termtracker.c - Simple 3-channel tracker in terminal
// Layout per row (3 channels, 16 rows):
// C-2 0 C01 --- 0 --- --- 0 --- I=01
// --- 0 --- --- 0 --- --- 0 --- W 00 00 F 00 00
// --- 0 --- --- 0 --- --- 0 ---   00 00   00 00
// --- 0 --- --- 0 --- --- 0 ---   00 00   00 00
// etc.
//
// Features:
// - 3 channels, 16 rows (0..F).
// - Arrow key navigation.
// - Note input with keyboard as piano (z/s/x/d/...).
// - Octave +/- with '-' and '='.
// - Instrument digit per cell: keys '0'..'9'.
// - Backspace deletes the note ('---') in current cell.
// - Right side shows current cell instrument as I=xx and dummy W/F tables.

#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#define NUM_CHANNELS   3
#define PATTERN_ROWS   16

typedef int8_t Note; // -1 = empty, 0..95 = semitone C-0..B-7

typedef struct {
    Note    note;          // -1 means "---"
    uint8_t instr;         // 0..9 for now
    char    cmd[4];        // 3-char command + null (e.g. "C01", "---")
} Cell;

typedef struct {
    Cell cell[PATTERN_ROWS][NUM_CHANNELS];
} Pattern;

typedef struct {
    int cursorRow;     // 0..15
    int cursorChan;    // 0..2
    int currentOctave; // 0..7
} EditorState;

// --------- global pattern + terminal state -------------

static struct termios g_orig_termios;
static Pattern g_pattern;

// --------- terminal raw mode helpers -------------------

static void disable_raw_mode(void) {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_orig_termios);
}

static void enable_raw_mode(void) {
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) == -1) {
        perror("tcgetattr");
        exit(1);
    }
    atexit(disable_raw_mode);

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON | ISIG);
    raw.c_iflag &= ~(IXON | ICRNL);
    raw.c_oflag &= ~(OPOST);
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == -1) {
        perror("tcsetattr");
        exit(1);
    }
}

// --------- note formatting & mapping -------------------

static void note_to_string(Note n, char out[4]) {
    if (n < 0) {
        strcpy(out, "---");
        return;
    }
    static const char *names[12] = {
        "C-", "C#", "D-", "D#",
        "E-", "F-", "F#", "G-",
        "G#", "A-", "A#", "B-"
    };
    int octave  = n / 12;
    int noteIdx = n % 12;
    out[0] = names[noteIdx][0];
    out[1] = names[noteIdx][1];
    out[2] = (octave >= 0 && octave <= 9) ? (char)('0' + octave) : '?';
    out[3] = '\0';
}

// keyboard -> note (tracker style on bottom row)
static Note note_from_key(char c, int octave) {
    int noteIdx = -1;
    switch (c) {
        case 'z': noteIdx = 0;  break; // C
        case 's': noteIdx = 1;  break; // C#
        case 'x': noteIdx = 2;  break; // D
        case 'd': noteIdx = 3;  break; // D#
        case 'c': noteIdx = 4;  break; // E
        case 'v': noteIdx = 5;  break; // F
        case 'g': noteIdx = 6;  break; // F#
        case 'b': noteIdx = 7;  break; // G
        case 'h': noteIdx = 8;  break; // G#
        case 'n': noteIdx = 9;  break; // A
        case 'j': noteIdx = 10; break; // A#
        case 'm': noteIdx = 11; break; // B
        default:  return -1;
    }
    int n = octave * 12 + noteIdx;
    if (n < 0 || n > 95) return -1;
    return (Note)n;
}

// --------- pattern init --------------------------------

static void init_pattern(Pattern *p) {
    for (int r = 0; r < PATTERN_ROWS; ++r) {
        for (int c = 0; c < NUM_CHANNELS; ++c) {
            p->cell[r][c].note  = -1;
            p->cell[r][c].instr = 0;
            strcpy(p->cell[r][c].cmd, "---");
        }
    }

    // Example: put C-2 0 C01 on row 0, chan 0 to match your sample.
    p->cell[0][0].note  = 2 + 12 * 2; // C-2? (we'll just pick some semitone ~C-2-ish)
    p->cell[0][0].instr = 0;
    strcpy(p->cell[0][0].cmd, "C01");
}

// --------- helpers -------------------------------------

static void clear_screen(void) {
    // clear + home + disable line wrap for nicer alignment
    printf("\x1b[2J\x1b[H\x1b[?7l");
}

static void hex2(uint8_t v, char out[3]) {
    const char *hex = "0123456789ABCDEF";
    out[0] = hex[(v >> 4) & 0xF];
    out[1] = hex[v & 0xF];
    out[2] = '\0';
}

// --------- drawing -------------------------------------

static void draw_ui(const Pattern *p, const EditorState *ed) {
    clear_screen();

    printf("Tiny term-tracker (3ch, 16 rows)\n");
    printf("Arrows: move  |  z/s/x/d/...: notes  |  -/=: octave (%d)  |  0-9: instr  |  Backspace: del note  |  q: quit\n\n",
           ed->currentOctave);

    // Header row for channels
    printf("    CH0          CH1          CH2\n");

    for (int r = 0; r < PATTERN_ROWS; ++r) {
        char rowHex[3];
        hex2((uint8_t)r, rowHex);

        char cursorMark = (r == ed->cursorRow) ? '>' : ' ';
        printf("%c%s ", cursorMark, rowHex);

        for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
            const Cell *cell = &p->cell[r][ch];
            char nstr[4];
            note_to_string(cell->note, nstr);
            printf("%3s %1d %3s", nstr, cell->instr, cell->cmd);
            if (ch < NUM_CHANNELS - 1) printf(" ");
        }

        // Right-side info block similar to your example
        if (r == 0) {
            const Cell *cur = &p->cell[ed->cursorRow][ed->cursorChan];
            printf(" I=%02d", cur->instr);
        } else if (r == 1) {
            printf(" W 00 00 F 00 00");
        } else if (r == 2) {
            printf("   00 00   00 00");
        } else if (r == 3) {
            printf("   00 00   00 00");
        }

        printf("\n");
    }

    fflush(stdout);
}

// --------- input (arrow keys, etc.) --------------------

static int read_key(void) {
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return -1;
    return (unsigned char)c;
}

enum {
    KEY_NONE = 0,
    KEY_ARROW_UP,
    KEY_ARROW_DOWN,
    KEY_ARROW_LEFT,
    KEY_ARROW_RIGHT,
    KEY_BACKSPACE,
    KEY_OTHER
};

typedef struct {
    int type;   // enum above
    int ch;     // raw character if type == KEY_OTHER
} KeyEvent;

static KeyEvent get_key_event(void) {
    KeyEvent ev = { KEY_NONE, 0 };
    int c = read_key();
    if (c < 0) return ev;

    if (c == 0x1B) { // ESC sequence for arrows
        char seq[2];
        // Try to read "[<A/B/C/D]"
        if (read(STDIN_FILENO, &seq[0], 1) <= 0) return ev;
        if (read(STDIN_FILENO, &seq[1], 1) <= 0) return ev;

        if (seq[0] == '[') {
            switch (seq[1]) {
                case 'A': ev.type = KEY_ARROW_UP;    return ev;
                case 'B': ev.type = KEY_ARROW_DOWN;  return ev;
                case 'C': ev.type = KEY_ARROW_RIGHT; return ev;
                case 'D': ev.type = KEY_ARROW_LEFT;  return ev;
            }
        }
        return ev;
    }

    if (c == 127 || c == '\b') {
        ev.type = KEY_BACKSPACE;
        return ev;
    }

    ev.type = KEY_OTHER;
    ev.ch   = c;
    return ev;
}

// --------- main loop -----------------------------------

int main(void) {
    init_pattern(&g_pattern);

    EditorState ed;
    ed.cursorRow     = 0;
    ed.cursorChan    = 0;
    ed.currentOctave = 2; // to match your C-2 idea

    enable_raw_mode();

    int running = 1;
    while (running) {
        draw_ui(&g_pattern, &ed);

        KeyEvent ev = get_key_event();
        if (ev.type == KEY_NONE) continue;

        if (ev.type == KEY_ARROW_UP) {
            if (ed.cursorRow > 0) ed.cursorRow--;
        } else if (ev.type == KEY_ARROW_DOWN) {
            if (ed.cursorRow < PATTERN_ROWS - 1) ed.cursorRow++;
        } else if (ev.type == KEY_ARROW_LEFT) {
            if (ed.cursorChan > 0) ed.cursorChan--;
        } else if (ev.type == KEY_ARROW_RIGHT) {
            if (ed.cursorChan < NUM_CHANNELS - 1) ed.cursorChan++;
        } else if (ev.type == KEY_BACKSPACE) {
            // delete note on current cell
            g_pattern.cell[ed.cursorRow][ed.cursorChan].note = -1;
        } else if (ev.type == KEY_OTHER) {
            char c = (char)ev.ch;

            if (c == 'q') {
                running = 0;
                continue;
            }

            // octave control
            if (c == '-') {
                if (ed.currentOctave > 0) ed.currentOctave--;
                continue;
            } else if (c == '=') {
                if (ed.currentOctave < 7) ed.currentOctave++;
                continue;
            }

            // instrument digit 0..9 for current cell
            if (c >= '0' && c <= '9') {
                g_pattern.cell[ed.cursorRow][ed.cursorChan].instr =
                    (uint8_t)(c - '0');
                continue;
            }

            // note entry with piano keys
            Note n = note_from_key((char)tolower((unsigned char)c),
                                   ed.currentOctave);
            if (n >= 0) {
                Cell *cell = &g_pattern.cell[ed.cursorRow][ed.cursorChan];
                cell->note = n;
                // If cmd is empty, leave it as-is; instrument stays whatever user set.
                // Move down automatically (tracker style)
                if (ed.cursorRow < PATTERN_ROWS - 1) {
                    ed.cursorRow++;
                }
                continue;
            }

            // (Command editing not implemented yet – we'll add a mode for CCC later.)
        }
    }

    // On exit, terminal raw mode is disabled by atexit
    return 0;
}
