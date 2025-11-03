#include <stdio.h>

#include "pico/stdio.h"
#include "pico/stdlib.h"

int main() {
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

    puts("picoSid-synth CDC stub ready");

    absolute_time_t next_toggle = make_timeout_time_ms(250);
    bool led_state = false;

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch >= 0) {
            putchar(ch);
            if (ch == '\r' || ch == '\n') {
                puts("pong");
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next_toggle) <= 0) {
            led_state = !led_state;
            gpio_put(led_pin, led_state);
            next_toggle = make_timeout_time_ms(250);
        }
    }
}
