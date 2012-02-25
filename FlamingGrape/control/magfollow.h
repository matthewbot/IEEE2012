#ifndef MAGFOLLOW_H_
#define MAGFOLLOW_H_

#include "hw/mag.h"

void magfollow_start(float heading);
void magfollow_stop();
void magfollow_tick();

struct MagCal {
	int16_t x_offset;
	int16_t y_offset;
	float y_scale;
};

#endif
