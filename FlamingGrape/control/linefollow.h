#ifndef LINEFOLLOW_H_
#define LINEFOLLOW_H_

#include "control/pid.h"
#include "hw/linesensor.h"
#include <stdint.h>

enum LineFollowFeature {
	FEATURE_NONE,
	FEATURE_INTERSECTION,
	FEATURE_NOLINE
};
void linefollow_printFeature(LineFollowFeature feature);

enum LineFollowTurn {
	TURN_NONE,
	TURN_LEFT,
	TURN_RIGHT
};
void linefollow_printTurn(LineFollowTurn turn);

struct LineFollowResults {
	float light[linesensor_count];
	bool thresh[linesensor_count];
	uint8_t thresh_count;
	float center;
	LineFollowTurn turn;
	LineFollowFeature feature;
};
void linefollow_printResults(const LineFollowResults &results);

void linefollow_start(float vel, float linepos=0);
void linefollow_stop();
bool linefollow_isDone();
void linefollow_waitDone();

LineFollowFeature linefollow_getLastFeature();
LineFollowTurn linefollow_getLastTurn();

void linefollow_setThresh(float thresh);
void linefollow_setGains(const PIDGains &gains);
void linefollow_setDebug(bool debug);
PIDGains linefollow_getGains();

LineFollowResults linefollow_readSensor();
void linefollow_waitLine();

void linefollow_tick();


#endif /* LINEFOLLOW_H_ */
