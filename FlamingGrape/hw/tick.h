#ifndef TICK_H
#define TICK_H

#include <stdint.h>

#define TICK_HZ 200
#define TICK_MS (1000/TICK_HZ)
#define TICK_DT (1.0f/TICK_HZ)

#define TICK_TIMHZ 4E6
#define TICK_TIMMAX (TICK_TIMHZ/TICK_HZ)

void tick_init();

void tick_wait();
void tick_halt();

uint16_t tick_getTimer();
uint32_t tick_getCount();

#endif
