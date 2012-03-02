#include "control/linefollow.h"
#include "control/motorcontrol.h"
#include "control/pid.h"
#include "control/drive.h"
#include "debug/debug.h"
#include "hw/tick.h"
#include <avr/pgmspace.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static PIDGains pidgains = {100, 0, 6, 0};
static float thresh = 3;

static volatile bool enabled;
static volatile float vel;
static volatile bool debug;
static volatile float linepos;
static volatile LineFollowFeature lastfeature;
static volatile LineFollowTurn lastturn;
static PIDState pidstate;

bool linefollow_start(float newvel, float newlinepos) {
	if (linefollow_readSensor().feature == FEATURE_NOLINE)
		return false;
	
	pid_initState(pidstate);
	vel = newvel;
	linepos = newlinepos;
	lastturn = TURN_NONE;
	lastfeature = FEATURE_NONE;
	enabled = true;
	return true;
}

void linefollow_stop() {
	if (enabled) {
		enabled = false;
		drive_stop();
	}
}

bool linefollow_isDone() {
	return !enabled;
}

void linefollow_waitDone() {
	while (enabled) { }
}

LineFollowFeature linefollow_getLastFeature() {
	return lastfeature;
}

LineFollowTurn linefollow_getLastTurn() {
	return lastturn;
}

static float pow4(float val) {
	float pow2 = val*val;
	return pow2*pow2;
}

LineFollowResults linefollow_readSensor() {	
	uint16_t readings[linesensor_count];
	linesensor_read(readings);
	
	LineFollowResults results;
	results.thresh_count = 0;
	float sum=0;
	float tot=0;
	
	for (int i=0; i<linesensor_count; i++) {
		float light;
		if (readings[i] > 500)
			light = 5E15 / pow4(1.0f+readings[i]); // maps line to ~35, dark to <<0
		else
			light = 0;
		results.light[i] = light;
		results.thresh[i] = light > thresh;
		
		if (results.thresh[i]) {
			results.thresh_count++;
		
			sum += i*light;
			tot += light;
		}
	}
	
	if (results.thresh_count)
		results.center = sum/(tot*(linesensor_count-1)/2.0) - 1;
	else
		results.center = 0;
		
	if (results.thresh[0]) {
		results.turn = TURN_LEFT;
	} else if (results.thresh[linesensor_count-1]) {
		results.turn = TURN_RIGHT;
	} else {
		results.turn = TURN_NONE;
	}

	if (results.thresh_count > 7) {
		results.feature = FEATURE_INTERSECTION;
	} else if (results.thresh_count == 0) { 
		results.feature = FEATURE_NOLINE;
	} else {
		results.feature = FEATURE_NONE;
	}
	
	return results;
}

void linefollow_waitLine(int left, int right) {
	while (true) {
		LineFollowResults results = linefollow_readSensor();
		for (int i=left; i<=right; i++)
			if (results.thresh[i])
				return;
		tick_wait();
	}
}

void linefollow_setThresh(float newthresh) {
	thresh = newthresh;
}

float linefollow_getThresh() {
	return thresh;
}

void linefollow_setGains(const PIDGains &newpidgains) {
	pidgains = newpidgains;
}

void linefollow_setDebug(bool newdebug) {
	debug = newdebug;
}

PIDGains linefollow_getGains() {
	return pidgains;
}

void linefollow_tick() {
	if (!enabled)
		return;
	
	LineFollowResults results = linefollow_readSensor();
	
	if (results.turn != TURN_NONE)
		lastturn = results.turn;
		
	if (results.feature != FEATURE_NONE) {
		lastfeature = results.feature;
		linefollow_stop();
		return;
	}
	
	PIDDebug piddebug;
	float error = results.center - linepos;
	float out = pid_update(pidstate, pidgains, error, TICK_DT, &piddebug);
	
	if (debug)
		pid_printDebug(out, error, piddebug);
	
	drive_steer(out, vel);
}

static char none_str[] PROGMEM = "NONE";
static char intersection_str[] PROGMEM = "INTERSECTION";
static char noline_str[] PROGMEM = "NOLINE";
static char left_str[] PROGMEM = "LEFT";
static char right_str[] PROGMEM = "RIGHT";

void linefollow_printFeature(LineFollowFeature feature) {
	static PGM_P table[] PROGMEM = {
		none_str,
		intersection_str,
		noline_str
	};
	printf_P((PGM_P)pgm_read_word(table + feature));
}

void linefollow_printTurn(LineFollowTurn turn) {
	static PGM_P table[] PROGMEM = {
		none_str,
		left_str,
		right_str
	};
	printf_P((PGM_P)pgm_read_word(table + turn));
}
