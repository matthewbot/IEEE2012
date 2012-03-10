#ifndef DEBUG_H_
#define DEBUG_H_

#include <stddef.h>
#include <stdint.h>

enum DebugLED {
	BOARD_LED,
	ERROR_LED, // red
	YELLOW_LED, // motorcontrol not hitting desired rps
	GREEN_LED,
	OTHERYELLOW_LED
};

void debug_init();
void debug_setLED(DebugLED led, bool on);

void debug_resetTimer();
uint16_t debug_getTimer(); // in us

void debug_println(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void debug_setEchoEnabled(bool enabled);

void debug_halt(const char *reason);

void debug_tick();

#endif
