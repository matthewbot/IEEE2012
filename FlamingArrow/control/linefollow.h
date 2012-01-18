#ifndef LINEFOLLOW_H_
#define LINEFOLLOW_H_

#include <stdint.h>
#include "hw/linesensor.h"

void linefollow_init();

struct LineFollowResults {
	float raw_light[linesensor_count];
	float raw_min;
	float light[linesensor_count];
	float squaresum;
	float squaretotal;
	float max;
	float steer; // range is [-1, +1]
};

void linefollow_computeResults(const uint16_t *readings, LineFollowResults &results);

void linefollow_bump(float offset=0);

#endif /* LINEFOLLOW_H_ */
