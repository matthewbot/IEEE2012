#ifndef MAG_H
#define MAG_H

#include <stdint.h>

void mag_init();

struct MagReading {
	int16_t x;
	int16_t y;
	int16_t z;
};

MagReading mag_getReading();

#endif
