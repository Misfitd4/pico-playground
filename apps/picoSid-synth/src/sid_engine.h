#ifndef PICO_SID_SYNTH_SID_ENGINE_H_
#define PICO_SID_SYNTH_SID_ENGINE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void sid_engine_init(uint32_t sample_rate_hz);
void sid_engine_note_on(uint8_t midi_note, uint8_t velocity);
void sid_engine_note_off(uint8_t midi_note);
void sid_engine_render_frame(int16_t *left, int16_t *right);
void sid_engine_queue_event(uint8_t chip, uint8_t addr, uint8_t value, uint32_t delta_cycles);
void sid_engine_set_channel_models(bool left_6581, bool right_6581);
void sid_engine_set_model(bool use_6581);
bool sid_engine_is_6581(void);
typedef struct {
    uint16_t voice_freq[3];
    uint8_t voice_control[3];
    uint8_t voice_envelope[3];
    uint16_t filter_cutoff;
    uint8_t filter_resonance;
    uint8_t filter_mode;
} sid_engine_monitor_t;

void sid_engine_get_monitor(sid_engine_monitor_t *out);
uint32_t sid_engine_get_queue_depth(void);
uint32_t sid_engine_get_dropped_event_count(void);
void sid_engine_reset_queue_state(void);

#ifdef __cplusplus
}
#endif

#endif  // PICO_SID_SYNTH_SID_ENGINE_H_
