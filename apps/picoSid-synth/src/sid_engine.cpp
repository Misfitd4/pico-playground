#include "sid_engine.h"

#include <math.h>
#include <stddef.h>
#include <climits>

#include "reSID16/sid.h"
#include "reSID16/siddefs.h"
#include "reSID_LUT.h"

extern "C" {
#include "exodecr.h"
}

#ifndef SID_LEFT_IS_6581
#define SID_LEFT_IS_6581 1
#endif

#ifndef SID_RIGHT_IS_6581
#define SID_RIGHT_IS_6581 0
#endif

namespace {

constexpr double kC64ClockHz = 985248.0;
constexpr uint8_t kAttackDecay = 0x11;      // Attack 1, Decay 1
constexpr uint8_t kReleaseRate = 0x04;      // Release rate
constexpr uint8_t kDefaultSustain = 0x0f;   // Sustained level (max)
constexpr uint8_t kWaveformSaw = 0x20;      // Sawtooth waveform bit
constexpr float kOutputGain = 1.5f;

struct VoiceState {
    bool active;
    uint8_t note;
    uint8_t velocity;
    uint32_t generation;
};

bool g_tables_ready = false;
SID16 *g_sids[2] = { nullptr, nullptr };
VoiceState g_voices[3] = {};
uint32_t g_voice_generation = 0;
double g_cycles_per_sample = 0.0;
double g_cycle_residual = 0.0;
uint32_t g_sample_rate_hz = 0;
chip_model g_channel_model[2] = {
    SID_LEFT_IS_6581 ? MOS6581 : MOS8580,
    SID_RIGHT_IS_6581 ? MOS6581 : MOS8580
};

struct TimedEvent {
    uint8_t chip_mask;
    uint8_t addr;
    uint8_t value;
    uint32_t delta;
};

constexpr size_t kEventQueueSize = 8192;
TimedEvent g_event_queue[kEventQueueSize];
size_t g_event_head = 0;
size_t g_event_tail = 0;
uint32_t g_cycles_to_next_event = UINT32_MAX;
uint32_t g_event_drop_count = 0;

inline size_t advance_index(size_t idx) { return (idx + 1) % kEventQueueSize; }
inline bool event_queue_empty() { return g_event_head == g_event_tail; }

void drop_oldest_event() {
    if (event_queue_empty()) return;
    TimedEvent dropped = g_event_queue[g_event_head];
    size_t next = advance_index(g_event_head);
    g_event_drop_count++;
    if (next != g_event_tail) {
        g_event_queue[next].delta += dropped.delta;
        g_event_head = next;
        g_cycles_to_next_event = g_event_queue[g_event_head].delta;
    } else {
        g_event_head = next;
        g_cycles_to_next_event = UINT32_MAX;
    }
}

void apply_event(const TimedEvent &ev) {
    uint8_t mask = ev.chip_mask & 0x3u;
    if (!mask) mask = 0x3u;  // default to both SIDs
    if ((mask & 0x1u) && g_sids[0]) {
        g_sids[0]->write(ev.addr & 0x1fu, ev.value);
    }
    if ((mask & 0x2u) && g_sids[1]) {
        g_sids[1]->write(ev.addr & 0x1fu, ev.value);
    }
}

bool pop_event(TimedEvent *out) {
    if (event_queue_empty()) return false;
    *out = g_event_queue[g_event_head];
    g_event_head = advance_index(g_event_head);
    if (event_queue_empty()) {
        g_cycles_to_next_event = UINT32_MAX;
    } else {
        g_cycles_to_next_event = g_event_queue[g_event_head].delta;
    }
    return true;
}

void apply_zero_delta_events() {
    while (!event_queue_empty() && g_event_queue[g_event_head].delta == 0) {
        TimedEvent ev;
        pop_event(&ev);
        apply_event(ev);
    }
}

uint16_t midi_note_to_sid(uint8_t midi_note) {
    const float note_frequency =
        440.0f * powf(2.0f, (static_cast<int>(midi_note) - 69) / 12.0f);
    double sid_value = (note_frequency * 16777216.0) / kC64ClockHz;
    if (sid_value < 0.0) {
        sid_value = 0.0;
    } else if (sid_value > 65535.0) {
        sid_value = 65535.0;
    }
    return static_cast<uint16_t>(sid_value + 0.5);
}

uint8_t velocity_to_sustain(uint8_t velocity) {
    if (!velocity) {
        return 0;
    }
    uint32_t scaled = static_cast<uint32_t>(velocity) * 15u + 63u;
    scaled /= 127u;
    if (scaled > 15u) {
        scaled = 15u;
    }
    return static_cast<uint8_t>(scaled);
}

void configure_voice_defaults(SID16 *sid, int voice_index) {
    const uint8_t base = static_cast<uint8_t>(voice_index * 7);
    sid->write(base + 0, 0);                           // Frequency low
    sid->write(base + 1, 0);                           // Frequency high
    sid->write(base + 2, 0);                           // Pulse width low
    sid->write(base + 3, 0x08);                        // Pulse width high (50%)
    sid->write(base + 4, kWaveformSaw);                // Waveform, gate off
    sid->write(base + 5, kAttackDecay);                // Attack/Decay
    sid->write(base + 6, (kDefaultSustain << 4) | kReleaseRate);
}

int find_voice_for_note(uint8_t midi_note) {
    for (int i = 0; i < 3; ++i) {
        if (g_voices[i].active && g_voices[i].note == midi_note) {
            return i;
        }
    }
    return -1;
}

int allocate_voice_slot() {
    int candidate = -1;
    for (int i = 0; i < 3; ++i) {
        if (!g_voices[i].active) {
            candidate = i;
            break;
        }
    }
    if (candidate >= 0) {
        return candidate;
    }
    uint32_t oldest_generation = g_voices[0].generation;
    candidate = 0;
    for (int i = 1; i < 3; ++i) {
        if (g_voices[i].generation < oldest_generation) {
            oldest_generation = g_voices[i].generation;
            candidate = i;
        }
    }
    return candidate;
}

void ensure_engine_initialised(uint32_t sample_rate_hz) {
    if (!g_tables_ready) {
        exo_decrunch(reinterpret_cast<const char *>(&reSID_LUTs_exo[reSID_LUTs_exo_size]),
                     reinterpret_cast<char *>(&reSID_LUTs[32768]));
        g_tables_ready = true;
    }

    for (int ch = 0; ch < 2; ++ch) {
        if (!g_sids[ch]) {
            g_sids[ch] = new SID16();
        }
    }

    g_sample_rate_hz = sample_rate_hz ? sample_rate_hz : 44100u;
    g_cycles_per_sample = kC64ClockHz / static_cast<double>(g_sample_rate_hz);
    g_cycle_residual = 0.0;

    for (int ch = 0; ch < 2; ++ch) {
        SID16 *sid = g_sids[ch];
        sid->set_chip_model(g_channel_model[ch]);
        sid->reset();
        sid->enable_filter(false);
        sid->enable_external_filter(false);
        sid->set_sampling_parameters(static_cast<float>(kC64ClockHz),
                                     SAMPLE_INTERPOLATE,
                                     static_cast<float>(g_sample_rate_hz));

        for (int voice = 0; voice < 3; ++voice) {
            configure_voice_defaults(sid, voice);
        }

        sid->write(0x15, 0x00);  // Filter cutoff low
        sid->write(0x16, 0x00);  // Filter cutoff high
        sid->write(0x17, 0x00);  // Resonance / routing disabled
        sid->write(0x18, 0x0f);  // Volume max, no filter
    }

    for (int i = 0; i < 3; ++i) {
        g_voices[i] = {};
    }
}

}  // namespace

void sid_engine_init(uint32_t sample_rate_hz) {
    ensure_engine_initialised(sample_rate_hz);
}

void sid_engine_note_on(uint8_t midi_note, uint8_t velocity) {
    int voice = find_voice_for_note(midi_note);
    if (voice < 0) {
        voice = allocate_voice_slot();
    }

    VoiceState &state = g_voices[voice];
    state.active = true;
    state.note = midi_note;
    state.velocity = velocity;
    state.generation = ++g_voice_generation;

    const uint16_t sid_freq = midi_note_to_sid(midi_note);
    const uint8_t base = static_cast<uint8_t>(voice * 7);

    for (SID16 *sid : g_sids) {
        if (!sid) continue;

        sid->write(base + 4, 0x08);  // TEST bit
        sid->write(base + 4, 0x00);

        sid->write(base + 0, sid_freq & 0xff);
        sid->write(base + 1, sid_freq >> 8);
        sid->write(base + 6, (velocity_to_sustain(velocity) << 4) | kReleaseRate);

        sid->write(base + 4, static_cast<uint8_t>(kWaveformSaw | 0x01));
    }
}

void sid_engine_note_off(uint8_t midi_note) {
    int voice = find_voice_for_note(midi_note);
    if (voice < 0) {
        return;
    }

    VoiceState &state = g_voices[voice];
    const uint8_t base = static_cast<uint8_t>(voice * 7);
    for (SID16 *sid : g_sids) {
        if (!sid) continue;
        sid->write(base + 4, kWaveformSaw);  // Clear gate, keep waveform
    }
    state.active = false;
}

void sid_engine_render_frame(int16_t *left, int16_t *right) {
    if (!left || !right) {
        if (left) {
            *left = 0;
        }
        if (right) {
            *right = 0;
        }
        return;
    }

    g_cycle_residual += g_cycles_per_sample;
    int cycles = static_cast<int>(g_cycle_residual);
    g_cycle_residual -= cycles;
    if (cycles < 1) {
        cycles = 1;
        g_cycle_residual = 0.0;
    }

    apply_zero_delta_events();

    int cycles_remaining = cycles;
    while (cycles_remaining > 0) {
        uint32_t run = static_cast<uint32_t>(cycles_remaining);
        if (!event_queue_empty() && g_cycles_to_next_event != UINT32_MAX && g_cycles_to_next_event < run) {
            run = g_cycles_to_next_event;
        }

        for (int ch = 0; ch < 2; ++ch) {
            SID16 *sid = g_sids[ch];
            if (!sid) continue;
            sid->clock(run);
        }

        cycles_remaining -= static_cast<int>(run);

        if (!event_queue_empty() && g_cycles_to_next_event != UINT32_MAX) {
            if (g_cycles_to_next_event > run) {
                g_cycles_to_next_event -= run;
            } else {
                g_cycles_to_next_event -= run;
                TimedEvent ev;
                if (pop_event(&ev)) {
                    apply_event(ev);
                }
                apply_zero_delta_events();
            }
        }
    }

    int32_t samples[2] = {0, 0};
    for (int ch = 0; ch < 2; ++ch) {
        SID16 *sid = g_sids[ch];
        if (!sid) continue;
        samples[ch] = sid->output();
    }

    auto clamp16 = [](int32_t value) -> int16_t {
        if (value > 32767) return 32767;
        if (value < -32768) return -32768;
        return static_cast<int16_t>(value);
    };

    int32_t amplified_left = static_cast<int32_t>(samples[0] * kOutputGain);
    int32_t amplified_right = static_cast<int32_t>(samples[1] * kOutputGain);
    *left = clamp16(amplified_left);
    *right = clamp16(amplified_right);
}

void sid_engine_queue_event(uint8_t chip_mask, uint8_t addr, uint8_t value, uint32_t delta_cycles) {
    size_t next_tail = advance_index(g_event_tail);
    if (next_tail == g_event_head) {
        drop_oldest_event();
        next_tail = advance_index(g_event_tail);
    }

    g_event_queue[g_event_tail].chip_mask = chip_mask;
    g_event_queue[g_event_tail].addr = addr;
    g_event_queue[g_event_tail].value = value;
    g_event_queue[g_event_tail].delta = delta_cycles;
    g_event_tail = next_tail;

    if (g_cycles_to_next_event == UINT32_MAX) {
        g_cycles_to_next_event = g_event_queue[g_event_head].delta;
        apply_zero_delta_events();
    }
}

void sid_engine_set_channel_models(bool left_6581, bool right_6581) {
    chip_model new_models[2] = {
        left_6581 ? MOS6581 : MOS8580,
        right_6581 ? MOS6581 : MOS8580
    };

    if (new_models[0] == g_channel_model[0] &&
        new_models[1] == g_channel_model[1]) {
        return;
    }

    g_channel_model[0] = new_models[0];
    g_channel_model[1] = new_models[1];
    ensure_engine_initialised(g_sample_rate_hz ? g_sample_rate_hz : 44100u);
}

void sid_engine_set_model(bool use_6581) {
    sid_engine_set_channel_models(use_6581, use_6581);
}

bool sid_engine_is_6581(void) {
    return g_channel_model[0] == MOS6581 && g_channel_model[1] == MOS6581;
}

void sid_engine_get_monitor(sid_engine_monitor_t *out) {
    if (!out) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        out->voice_freq[i] = 0;
        out->voice_control[i] = 0;
        out->voice_envelope[i] = 0;
    }
    out->filter_cutoff = 0;
    out->filter_resonance = 0;
    out->filter_mode = 0;

    SID16 *sid = g_sids[0];
    if (!sid) {
        return;
    }

    SID16::State state = sid->read_state();
    for (int voice = 0; voice < 3; ++voice) {
        uint8_t base = static_cast<uint8_t>(voice * 7);
        uint16_t freq = (static_cast<uint16_t>(state.sid_register[base + 1]) << 8) |
                        static_cast<uint16_t>(state.sid_register[base + 0]);
        out->voice_freq[voice] = freq;
        out->voice_control[voice] = state.sid_register[base + 4];
        out->voice_envelope[voice] = state.envelope_counter[voice];
    }

    uint16_t cutoff = (static_cast<uint16_t>(state.sid_register[0x16] & 0x7Fu) << 3) |
                      static_cast<uint16_t>(state.sid_register[0x15] & 0x07u);
    out->filter_cutoff = cutoff;
    out->filter_resonance = (state.sid_register[0x17] >> 4) & 0x0Fu;
    out->filter_mode = state.sid_register[0x17] & 0x0Fu;
}

uint32_t sid_engine_get_queue_depth(void) {
    size_t head = g_event_head;
    size_t tail = g_event_tail;
    if (tail >= head) {
        return static_cast<uint32_t>(tail - head);
    }
    return static_cast<uint32_t>(kEventQueueSize - head + tail);
}

uint32_t sid_engine_get_dropped_event_count(void) {
    return g_event_drop_count;
}

void sid_engine_reset_queue_state(void) {
    g_event_head = 0;
    g_event_tail = 0;
    g_cycles_to_next_event = UINT32_MAX;
    g_event_drop_count = 0;
    g_cycle_residual = 0.0;
}
