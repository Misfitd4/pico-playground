#include "siddler_audio.h"

#include <stddef.h>

#include "pico/audio_i2s.h"
#include "pico/stdlib.h"

#include "sid_engine.h"

#ifndef SIDDLER_AUDIO_SAMPLE_RATE
#define SIDDLER_AUDIO_SAMPLE_RATE 44100u
#endif

#ifndef SIDDLER_AUDIO_BUFFER_SAMPLES
#define SIDDLER_AUDIO_BUFFER_SAMPLES 256u
#endif

static audio_format_t siddler_audio_format = {
	.sample_freq = SIDDLER_AUDIO_SAMPLE_RATE,
	.format = AUDIO_BUFFER_FORMAT_PCM_S16,
	.channel_count = 2,
};

static struct audio_buffer_format siddler_producer_format = {
	.format = &siddler_audio_format,
	.sample_stride = 4,
};

static struct audio_buffer_pool *siddler_audio_pool = NULL;
static bool siddler_audio_enabled = false;

static void siddler_audio_fill_buffer(struct audio_buffer *buffer) {
	int16_t *samples = (int16_t *) buffer->buffer->bytes;
	for (uint i = 0; i < buffer->max_sample_count; ++i) {
		int16_t left = 0;
		int16_t right = 0;
		sid_engine_render_frame(&left, &right);
		samples[(i << 1) + 0] = left;
		samples[(i << 1) + 1] = right;
	}
	buffer->sample_count = buffer->max_sample_count;
}

static void siddler_audio_prime_buffers(void) {
	if (!siddler_audio_pool) {
		return;
	}
	for (int i = 0; i < 2; ++i) {
		struct audio_buffer *buffer = take_audio_buffer(siddler_audio_pool, false);
		if (!buffer) {
			break;
		}
		siddler_audio_fill_buffer(buffer);
		give_audio_buffer(siddler_audio_pool, buffer);
	}
}

bool siddler_audio_init(void) {
	if (siddler_audio_enabled) {
		return true;
	}

	if (!siddler_audio_pool) {
		siddler_audio_pool = audio_new_producer_pool(&siddler_producer_format, 3, SIDDLER_AUDIO_BUFFER_SAMPLES);
		if (!siddler_audio_pool) {
			return false;
		}
	}

	struct audio_i2s_config config = {
		.data_pin = PICO_AUDIO_I2S_DATA_PIN,
		.clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
		.dma_channel = 6,
		.pio_sm = 0,
	};

	const struct audio_format *output_format = audio_i2s_setup(&siddler_audio_format, &config);
	if (!output_format) {
		return false;
	}

	if (!audio_i2s_connect(siddler_audio_pool)) {
		audio_i2s_set_enabled(false);
		return false;
	}

	audio_i2s_set_enabled(true);
	siddler_audio_enabled = true;

	sid_engine_init(siddler_audio_format.sample_freq);
	sid_engine_set_channel_models(true, true);

	siddler_audio_prime_buffers();
	return true;
}

void siddler_audio_shutdown(void) {
	if (siddler_audio_enabled) {
		audio_i2s_set_enabled(false);
		siddler_audio_enabled = false;
	}
}

void siddler_audio_reset_state(void) {
	if (!siddler_audio_enabled) {
		return;
	}
	sid_engine_init(siddler_audio_format.sample_freq);
}

void siddler_audio_queue_event(uint8_t chip_mask, uint8_t addr, uint8_t value, uint32_t delta_cycles) {
	if (!siddler_audio_enabled) {
		return;
	}
	sid_engine_queue_event(chip_mask, (uint8_t) (addr & 0x1Fu), value, delta_cycles);
}

void siddler_audio_task(void) {
	if (!siddler_audio_pool || !siddler_audio_enabled) {
		return;
	}
	struct audio_buffer *buffer = take_audio_buffer(siddler_audio_pool, false);
	if (!buffer) {
		return;
	}
	siddler_audio_fill_buffer(buffer);
	give_audio_buffer(siddler_audio_pool, buffer);
}
