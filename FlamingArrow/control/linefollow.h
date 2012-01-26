#ifndef LINEFOLLOW_H_
#define LINEFOLLOW_H_

#include <stdint.h>
#include "hw/linesensor.h"

void linefollow_init();

enum LineFollowFeature {
	FEATURE_NONE,
	FEATURE_INTERSECTION,
	FEATURE_RIGHTTURN,
	FEATURE_LEFTTURN,
	FEATURE_NOLINE
};

struct LineFollowResults {
	float light[linesensor_count];
	float light_max;
	float squaresum;
	float squaretotal;
	float steer; // range is [-1, +1]
	
	LineFollowFeature feature;
};

void linefollow_computeResults(const uint16_t *readings, LineFollowResults &results);

void linefollow_bump(float offset=0);

void linefollow_wait_line();

#endif /* LINEFOLLOW_H_ */
