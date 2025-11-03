#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#define HEARTBEAT_FAST_MS 100
#define HEARTBEAT_SLOW_MS 400

int main(void) {
	stdio_init_all();

	const uint led_pin = PICO_DEFAULT_LED_PIN;
	bool led_on = false;
	gpio_init(led_pin);
	gpio_set_dir(led_pin, GPIO_OUT);
	gpio_put(led_pin, led_on);

	bool connected = stdio_usb_connected();
	absolute_time_t next_blink = make_timeout_time_ms(connected ? HEARTBEAT_FAST_MS : HEARTBEAT_SLOW_MS);

	if (connected) {
		printf("\r\nusb_serial_echo ready\r\n> ");
		fflush(stdout);
	}

	while (true) {
		bool now_connected = stdio_usb_connected();
		if (now_connected && !connected) {
			printf("\r\nusb_serial_echo ready\r\n> ");
			fflush(stdout);
			next_blink = make_timeout_time_ms(HEARTBEAT_FAST_MS);
		} else if (!now_connected && connected) {
			printf("\r\n[USB disconnected]\r\n");
			fflush(stdout);
			next_blink = make_timeout_time_ms(HEARTBEAT_SLOW_MS);
		}
		connected = now_connected;

		int ch = getchar_timeout_us(0);
		if (ch >= 0) {
			if (ch == '\r' || ch == '\n') {
				printf("\r\n> ");
			} else if (ch == 0x04) {
				printf("\r\n^D ignored\r\n> ");
			} else {
				putchar(ch);
				if (ch == '\b') {
					putchar(' ');
					putchar('\b');
				}
			}
			fflush(stdout);
		}

		if (absolute_time_diff_us(get_absolute_time(), next_blink) <= 0) {
			led_on = !led_on;
			gpio_put(led_pin, led_on);
			next_blink = make_timeout_time_ms(connected ? HEARTBEAT_FAST_MS : HEARTBEAT_SLOW_MS);
		}

		tight_loop_contents();
	}

	return 0;
}
