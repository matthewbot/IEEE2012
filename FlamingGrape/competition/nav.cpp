#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "control/magfollow.h"
#include "hw/adc.h"
#include "hw/tick.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include "debug/debug.h"
#include "util.h"
#include <stdio.h>
#include <util/delay.h>

static bool pause;

void nav_magGo(float heading_deg, float dist) {
	if (dist > 0) {
		magfollow_start(60, degtorad(heading_deg));
	} else {
		magfollow_start(-60, degtorad(heading_deg));
	}
	drive_waitDist(dist);
	magfollow_stop();
}

bool nav_linefollowIntersection() {
	while (true) {
		if (!nav_linefollow())
			return false;
		
		if (linefollow_getLastFeature() == FEATURE_INTERSECTION)
			return true; 
		else if (linefollow_getLastTurn() == TURN_LEFT) {
			drive_lturn(50, DM_BANG);
			linefollow_waitLine(2, 7);
		} else if (linefollow_getLastTurn() == TURN_RIGHT) {
			drive_rturn(50, DM_BANG);
			linefollow_waitLine(0, 5);
		} else
			return false;
	}
}

bool nav_linefollowTurns(int turncount, float offset) {
	while (true) {
		if (!nav_linefollow(offset))
			return false;
		
		if (linefollow_getLastTurn() == TURN_LEFT) {
			printf("Got left\n");
			drive_lturnDeg(50, 70, DM_BANG);
			if (--turncount <= 0)
				return true;
		} else if (linefollow_getLastTurn() == TURN_RIGHT) {
			printf("Got right\n");
			drive_rturnDeg(50, 70, DM_BANG);
			if (--turncount <= 0)
				return true;
		} else {
			return false;
		}
	}
}

bool nav_linefollowRange(float range) {
	if (!linefollow_start(60))
		return false;
	_delay_ms(100);
	
	float dist=0;
	while (!linefollow_isDone()) {
		float reading = adc_sampleRangeFinder(ADC_FRONT_LEFT_RANGE);
		dist = .9f*dist + .1f*reading;
		if (dist < range) {
			linefollow_stop();
			return true;
		}
	}
	
	return false;
}

static const float wheel_circumference = 16.3; // TODO drive library needs a distance measuring interface

bool nav_linefollowDist(float dist) {
	if (!linefollow_start(60))
		return false;
	_delay_ms(100);
	
	uint16_t leftenc = enc_get(MOTOR_LEFT);
	uint16_t rightenc = enc_get(MOTOR_RIGHT);
	
	while (!linefollow_isDone()) {
		int16_t leftdiff = enc_diff(enc_get(MOTOR_LEFT), leftenc);
		int16_t rightdiff = enc_diff(enc_get(MOTOR_RIGHT), rightenc);
		float curdist = (leftdiff + rightdiff) * (wheel_circumference/2) / enc_per_rotation;
		
		if (curdist >= dist) {
			linefollow_stop();
			return true;
		}
	}
	
	return false;	
}

bool nav_linefollow(float offset) {
	if (!linefollow_start(60, offset))
		return false;
	linefollow_waitDone();
	return true;
}

bool nav_waitLineDist(int left, int right, float dist) {
	uint16_t leftenc = enc_get(MOTOR_LEFT);
	uint16_t rightenc = enc_get(MOTOR_RIGHT);
	
	while (!linefollow_getLine(left, right)) {
		int16_t leftdiff = enc_diff(enc_get(MOTOR_LEFT), leftenc);
		int16_t rightdiff = enc_diff(enc_get(MOTOR_RIGHT), rightenc);
		float curdist = (leftdiff + rightdiff) * (wheel_circumference/2) / enc_per_rotation;
		
		if (curdist >= dist) {
			linefollow_stop();
			return false;
		}
	}
	
	return true;
}

void nav_setPauseEnabled(bool newpause) {
	pause = newpause;
}

void nav_pause() {
	if (pause)
		_delay_ms(1000);
}

