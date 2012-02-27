#ifndef MAGFOLLOW_H_
#define MAGFOLLOW_H_

#include "hw/mag.h"

float magfollow_getHeading();
void magfollow_setHeading(float heading);

void magfollow_start(float vel, float heading);
void magfollow_stop();
void magfollow_tick();

void magfollow_setDebug(bool debug);

struct MagCal {
	int16_t x_offset;
	int16_t y_offset;
	float y_scale;
};

void magfollow_setCal(const MagCal &magcal);
const MagCal &magfollow_getCal();

#endif
