// tracker.c
#include <SDL2/SDL.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int8_t Note; // -1 = none, 0..95 = semitone

typedef struct {
    Note    note;
    uint8_t instr;
    uint8_t effect;
    uint8_t effectParam;
} Cell;

#define NUM_CHANNELS 3
#define PATTERN_ROWS 64

typedef struct {
    Cell row[PATTERN_ROWS][NUM_CHANNELS];
} Pattern;

typedef struct {
    uint8_t waveform;   // 0=pulse,1=saw,2=tri,3=noise
    int8_t  transpose;  // semitones
    uint8_t length;     // ticks
} WaveStep;

#define WAVETABLE_STEPS 32

typedef struct {
    WaveStep steps[WAVETABLE_STEPS];
    uint8_t  length;
} WaveTable;

typedef struct {
    uint8_t cutoff;     // 0..255
    uint8_t resonance;  // 0..255
    uint8_t length;     // ticks
} FilterStep;

#define FILTERTABLE_STEPS 32

typedef struct {
    FilterStep steps[FILTERTABLE_STEPS];
    uint8_t    length;
} FilterTable;

typedef struct {
    char     name[16];
    uint8_t  waveTableIndex;
    uint8_t  filterTableIndex;
    uint8_t  attack, decay, sustain, release;
    uint8_t  volume; // 0..64
} Instrument;

#define MAX_INSTRUMENTS 16
#define MAX_PATTERNS    16
#define MAX_ORDERS      64

typedef struct {
    uint8_t numPatterns;
    uint8_t numOrders;
    uint8_t orderList[MAX_ORDERS];

    uint8_t  speed; // ticks per row
    uint16_t bpm;

    Instrument  instruments[MAX_INSTRUMENTS];
    uint8_t     numInstruments;

    WaveTable   waveTables[8];
    FilterTable filterTables[8];

    Pattern     patterns[MAX_PATTERNS];
} Song;

// ===================== ENGINE STATE =====================

typedef struct {
    // timing
    double   samplesPerTick;
    double   tickSampleCounter;
    uint8_t  tick;        // 0..speed-1
    uint8_t  row;         // 0..PATTERN_ROWS-1
    uint8_t  order;       // 0..numOrders-1

    // per-channel runtime state
    struct {
        Note    note;
        float   phase;
        float   freq;
        uint8_t instrIndex;

        // wave/filter table state
        uint8_t wavePos;
        uint8_t waveTickLeft;

        uint8_t filtPos;
        uint8_t filtTickLeft;

        float   volume; // 0..1
    } ch[NUM_CHANNELS];

    Song *song;
    int   songEnd;
} EngineState;

static EngineState g_engine;

// ============ HELPER: note->frequency =================

static float note_to_freq(Note n) {
    if (n < 0) return 0.0f;
    // Let note 60 = C-5 ~ 523.25 Hz for convenience
    const float base = 523.25f;
    int delta = (int)n - 60;
    return base * powf(2.0f, delta / 12.0f);
}

// ============ ADVANCE WAVE/FILTER TABLES ===============

static void advance_wave_table(EngineState *e, int chIndex) {
    Song *s = e->song;
    auto *ch = &e->ch[chIndex];
    if (ch->instrIndex == 0xFF) return;

    Instrument *inst = &s->instruments[ch->instrIndex];
    if (inst->waveTableIndex >= 8) return;

    WaveTable *wt = &s->waveTables[inst->waveTableIndex];
    if (wt->length == 0) return;

    if (ch->waveTickLeft == 0) {
        ch->wavePos = (ch->wavePos + 1) % wt->length;
        ch->waveTickLeft = wt->steps[ch->wavePos].length;
    } else {
        ch->waveTickLeft--;
    }

    WaveStep *step = &wt->steps[ch->wavePos];
    Note n = ch->note;
    if (n >= 0) n += step->transpose;
    ch->freq = note_to_freq(n);
    // waveform selection is ignored in this minimal example
}

static void advance_filter_table(EngineState *e, int chIndex) {
    Song *s = e->song;
    auto *ch = &e->ch[chIndex];
    if (ch->instrIndex == 0xFF) return;

    Instrument *inst = &s->instruments[ch->instrIndex];
    if (inst->filterTableIndex >= 8) return;

    FilterTable *ft = &s->filterTables[inst->filterTableIndex];
    if (ft->length == 0) return;

    if (ch->filtTickLeft == 0) {
        ch->filtPos = (ch->filtPos + 1) % ft->length;
        ch->filtTickLeft = ft->steps[ch->filtPos].length;
    } else {
        ch->filtTickLeft--;
    }

    // For now we ignore cutoff/resonance in the synth;
    // they can later modulate a simple low-pass filter.
}

// ============ PROCESS ROW ==============================

static void process_row(EngineState *e) {
    Song *s = e->song;
    if (e->order >= s->numOrders) {
        e->songEnd = 1;
        return;
    }

    uint8_t patIndex = s->orderList[e->order];
    if (patIndex >= s->numPatterns) {
        e->songEnd = 1;
        return;
    }

    Pattern *pat = &s->patterns[patIndex];

    for (int chIndex = 0; chIndex < NUM_CHANNELS; ++chIndex) {
        Cell *cell = &pat->row[e->row][chIndex];
        if (cell->note >= 0 && cell->note <= 95 && cell->instr > 0) {
            // Note on
            auto *ch = &e->ch[chIndex];
            ch->note       = cell->note;
            ch->instrIndex = cell->instr - 1;
            ch->wavePos    = 0;
            ch->waveTickLeft = 0;
            ch->filtPos    = 0;
            ch->filtTickLeft = 0;

            Instrument *inst = &s->instruments[ch->instrIndex];
            ch->freq   = note_to_freq(ch->note);
            ch->volume = inst->volume / 64.0f;
        }

        // Effects can go here (arpeggio, slide, etc.) based on cell->effect
    }
}

// ============ ADVANCE TICK =============================

static void advance_tick(EngineState *e) {
    Song *s = e->song;

    if (e->tick == 0) {
        process_row(e);
    }

    // Advance per-channel tables every tick
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        advance_wave_table(e, i);
        advance_filter_table(e, i);
    }

    // Next tick
    e->tick++;
    if (e->tick >= s->speed) {
        e->tick = 0;
        e->row++;
        if (e->row >= PATTERN_ROWS) {
            e->row = 0;
            e->order++;
            if (e->order >= s->numOrders) {
                e->songEnd = 1;
            }
        }
    }
}

// ============ AUDIO CALLBACK ===========================

static void audio_callback(void *userdata, Uint8 *stream, int len) {
    EngineState *e = (EngineState *)userdata;
    int16_t *out = (int16_t *)stream;
    int frames = len / (2 * sizeof(int16_t)); // stereo 16-bit

    for (int i = 0; i < frames; ++i) {
        if (!e->songEnd) {
            e->tickSampleCounter += 1.0;
            if (e->tickSampleCounter >= e->samplesPerTick) {
                e->tickSampleCounter -= e->samplesPerTick;
                advance_tick(e);
            }
        }

        float mix = 0.0f;
        for (int chIndex = 0; chIndex < NUM_CHANNELS; ++chIndex) {
            auto *ch = &e->ch[chIndex];
            if (ch->freq > 0.0f && ch->volume > 0.0f) {
                float phaseInc = ch->freq / 44100.0f;
                ch->phase += phaseInc;
                if (ch->phase >= 1.0f) ch->phase -= 1.0f;

                // simple pulse wave 50%
                float sample = (ch->phase < 0.5f ? 1.0f : -1.0f) * ch->volume;
                mix += sample;
            }
        }

        // soft clip
        if (mix > 1.0f) mix = 1.0f;
        if (mix < -1.0f) mix = -1.0f;

        int16_t s = (int16_t)(mix * 30000.0f);
        out[2 * i + 0] = s;
        out[2 * i + 1] = s;
    }
}

// ============ TEST SONG SETUP ==========================

static void build_test_song(Song *s) {
    memset(s, 0, sizeof(*s));

    s->numPatterns = 1;
    s->numOrders   = 1;
    s->orderList[0] = 0;
    s->speed = 6;
    s->bpm   = 125;

    // Instrument 1
    s->numInstruments = 1;
    Instrument *inst = &s->instruments[0];
    strcpy(inst->name, "Lead");
    inst->waveTableIndex   = 0;
    inst->filterTableIndex = 0;
    inst->attack = 2;
    inst->decay  = 4;
    inst->sustain = 40;
    inst->release = 4;
    inst->volume  = 48;

    // Simple wave table: transpose pattern
    WaveTable *wt = &s->waveTables[0];
    wt->length = 3;
    wt->steps[0].waveform = 0; wt->steps[0].transpose = 0;  wt->steps[0].length = 1;
    wt->steps[1].waveform = 0; wt->steps[1].transpose = 4;  wt->steps[1].length = 1;
    wt->steps[2].waveform = 0; wt->steps[2].transpose = 7;  wt->steps[2].length = 1;

    // Dummy filter table (unused)
    FilterTable *ft = &s->filterTables[0];
    ft->length = 1;
    ft->steps[0].cutoff = 128;
    ft->steps[0].resonance = 64;
    ft->steps[0].length = 1;

    // Pattern 0: simple arpeggiated chord spread over 3 channels
    Pattern *p = &s->patterns[0];
    for (int r = 0; r < PATTERN_ROWS; ++r) {
        for (int c = 0; c < NUM_CHANNELS; ++c) {
            p->row[r][c].note = -1;
            p->row[r][c].instr = 0;
            p->row[r][c].effect = 0;
            p->row[r][c].effectParam = 0;
        }
    }

    // Channel 0: C-4 every 8 rows
    for (int r = 0; r < PATTERN_ROWS; r += 8) {
        p->row[r][0].note = 48; // C-4
        p->row[r][0].instr = 1;
    }

    // Channel 1: E-4 staggered
    for (int r = 4; r < PATTERN_ROWS; r += 8) {
        p->row[r][1].note = 52; // E-4
        p->row[r][1].instr = 1;
    }

    // Channel 2: G-4 staggered
    for (int r = 2; r < PATTERN_ROWS; r += 8) {
        p->row[r][2].note = 55; // G-4
        p->row[r][2].instr = 1;
    }
}

// ============ MAIN =====================================

int main(int argc, char **argv) {
    (void)argc; (void)argv;

    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    Song song;
    build_test_song(&song);

    EngineState *e = &g_engine;
    memset(e, 0, sizeof(*e));
    e->song = &song;

    // Timing: classic tracker tick length = (2.5 * speed) / bpm seconds
    double tickLenSec = (2.5 * song.speed) / (double)song.bpm; // ProTracker-ish formula
    double sampleRate = 44100.0;
    e->samplesPerTick = tickLenSec * sampleRate;

    for (int i = 0; i < NUM_CHANNELS; ++i) {
        e->ch[i].note       = -1;
        e->ch[i].instrIndex = 0xFF;
        e->ch[i].phase      = 0.0f;
        e->ch[i].volume     = 0.0f;
    }

    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = (int)sampleRate;
    want.format = AUDIO_S16SYS;
    want.channels = 2;
    want.samples = 512;
    want.callback = audio_callback;
    want.userdata = e;

    if (SDL_OpenAudio(&want, &have) < 0) {
        fprintf(stderr, "SDL_OpenAudio failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_PauseAudio(0);
    printf("Playing test song... press Ctrl+C to quit.\n");

    // Simple loop; we could watch e->songEnd and exit after one playthrough
    while (!e->songEnd) {
        SDL_Delay(100);
    }

    SDL_CloseAudio();
    SDL_Quit();
    return 0;
}
