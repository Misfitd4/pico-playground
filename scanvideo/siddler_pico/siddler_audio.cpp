#include "siddler_audio.h"

#include <stddef.h>

#include "pico/audio_i2s.h"
#include "pico/stdlib.h"

#include "sid_engine.h"

#ifndef SIDDLER_AUDIO_SAMPLE_RATE
#define SIDDLER_AUDIO_SAMPLE_RATE 44100u
#endif

#ifndef SIDDLER_AUDIO_BUFFER_SAMPLES
#define SIDDLER_AUDIO_BUFFER_SAMPLES 96u
#endif

#ifndef SIDDLER_AUDIO_DMA_CHANNEL
#define SIDDLER_AUDIO_DMA_CHANNEL 6u
#endif

#ifndef SIDDLER_AUDIO_PIO_SM
#if PICO_AUDIO_I2S_PIO == 0
#define SIDDLER_AUDIO_PIO_SM 2u
#else
#define SIDDLER_AUDIO_PIO_SM 0u
#endif
#endif

#ifndef SIDDLER_AUDIO_TEST_TONE
#define SIDDLER_AUDIO_TEST_TONE 0
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
#if SIDDLER_AUDIO_TEST_TONE
		static uint32_t phase = 0;
		const uint32_t step = (uint32_t) ((uint64_t)SIDDLER_AUDIO_SAMPLE_RATE * (1u << 16) / 440u);
		phase += step;
		int16_t sample = (int16_t) (phase >> 16) - 32768;
		left = sample;
		right = sample;
#else
        sid_engine_render_frame(&left, &right);  // Use sid_engine_set_master_volume() to tweak the master level.
#endif
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
		siddler_audio_pool = audio_new_producer_pool(&siddler_producer_format, 8, SIDDLER_AUDIO_BUFFER_SAMPLES);
		if (!siddler_audio_pool) {
			return false;
		}
	}

	struct audio_i2s_config config = {
		.data_pin = PICO_AUDIO_I2S_DATA_PIN,
		.clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
		.dma_channel = SIDDLER_AUDIO_DMA_CHANNEL,
		.pio_sm = SIDDLER_AUDIO_PIO_SM,
	};

	const struct audio_format *output_format = audio_i2s_setup(&siddler_audio_format, &config);
	if (!output_format) {
		return false;
	}

	if (!audio_i2s_connect_extra(siddler_audio_pool, false, 2, 96, NULL)) {
		audio_i2s_set_enabled(false);
		return false;
	}

	audio_i2s_set_enabled(true);
	siddler_audio_enabled = true;

	sid_engine_reset_queue_state();
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
	sid_engine_reset_queue_state();
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
