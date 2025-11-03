#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define SQUARE_WAVE_PIN 26
#define HALF_PERIOD_US 500  // 1 kHz square wave

int main(void) {
	set_sys_clock_khz(120000, true);
	gpio_init(SQUARE_WAVE_PIN);
	gpio_set_dir(SQUARE_WAVE_PIN, GPIO_OUT);

	while (true) {
		gpio_put(SQUARE_WAVE_PIN, 1);
		sleep_us(HALF_PERIOD_US);
		gpio_put(SQUARE_WAVE_PIN, 0);
		sleep_us(HALF_PERIOD_US);
	}
}
