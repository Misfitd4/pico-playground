#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"

static const char *note_names[] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

typedef struct {
    uint8_t status;
    uint8_t data[2];
    uint8_t count;
} midi_parser_t;

static void reset_parser(midi_parser_t *p) {
    p->status = 0;
    p->count = 0;
}

static void handle_note_event(uint8_t status, uint8_t note, uint8_t velocity) {
    uint8_t channel = status & 0x0F;
    bool note_on = ((status & 0xF0) == 0x90) && velocity > 0;
    const char *name = note_names[note % 12];
    int octave = (note / 12) - 1;
    float freq = 440.0f * powf(2.0f, ((int)note - 69) / 12.0f);

    printf("%s ch=%u note=%s%d (%u) vel=%u freq=%.2fHz\n",
           note_on ? "NOTE ON " : "NOTE OFF",
           channel + 1, name, octave, note, velocity, freq);
}

static void process_midi_byte(midi_parser_t *parser, uint8_t byte) {
    if (byte & 0x80) {
        parser->status = byte;
        parser->count = 0;
        return;
    }

    if (!parser->status) {
        return;
    }

    uint8_t type = parser->status & 0xF0;
    if (type == 0x80 || type == 0x90) {
        parser->data[parser->count++] = byte;
        if (parser->count == 2) {
            handle_note_event(parser->status, parser->data[0], parser->data[1]);
            parser->count = 0;
        }
    }
}

int main(void) {
    const uint led_pin = PICO_DEFAULT_LED_PIN;
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_put(led_pin, 0);

    if (!stdio_init_all()) {
        while (true) {
            gpio_put(led_pin, 1);
            sleep_ms(150);
            gpio_put(led_pin, 0);
            sleep_ms(150);
        }
    }

    puts("usb_serial_probe MIDI monitor ready");

    midi_parser_t parser;
    reset_parser(&parser);

    absolute_time_t next_blink = make_timeout_time_ms(200);
    bool led_state = false;

    while (true) {
        int byte = getchar_timeout_us(0);
        if (byte >= 0) {
            process_midi_byte(&parser, (uint8_t)byte);
        }

        if (absolute_time_diff_us(get_absolute_time(), next_blink) <= 0) {
            led_state = !led_state;
            gpio_put(led_pin, led_state);
            next_blink = make_timeout_time_ms(200);
        }
    }
}
