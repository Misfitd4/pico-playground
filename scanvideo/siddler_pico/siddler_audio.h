#ifndef SIDDLE_AUDIO_H_
#define SIDDLE_AUDIO_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool siddler_audio_init(void);
void siddler_audio_shutdown(void);
void siddler_audio_reset_state(void);
void siddler_audio_queue_event(uint8_t chip_mask, uint8_t addr, uint8_t value, uint32_t delta_cycles);
void siddler_audio_task(void);

#ifdef __cplusplus
}
#endif

#endif /* SIDDLE_AUDIO_H_ */
