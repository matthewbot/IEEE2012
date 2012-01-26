#ifndef DEBUG_H_
#define DEBUG_H_

#include <stddef.h>

void debug_init();
void debug_setLED(bool on);

void debug_resetTimer();
uint16_t debug_getTimer();

#endif
