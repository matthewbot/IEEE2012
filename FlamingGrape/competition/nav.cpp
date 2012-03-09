#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "control/magfollow.h"
#include "hw/adc.h"
#include "hw/tick.h"
#include "debug/debug.h"
#include "util.h"
#include <stdio.h>
#include <util/delay.h>

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

bool nav_linefollowTurns(int turncount) {
	while (true) {
		if (!nav_linefollow())
			return false;
		
		if (linefollow_getLastTurn() == TURN_LEFT) {
			if (--turncount <= 0)
				return true;
			drive_lturnDeg(50, 70, DM_BANG);
		} else if (linefollow_getLastTurn() == TURN_RIGHT) {
			if (--turncount <= 0)
				return true;
			drive_rturnDeg(50, 70, DM_BANG);
		} else {
			return false;
		}
	}
}

bool nav_linefollowRange(float range) {
	if (!linefollow_start(60))
		return false;
	_delay_ms(100);
	
	while (!linefollow_isDone()) {
		float reading = adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE);
		if (reading < range) {
			linefollow_stop();
			return true;
		}
	}
	
	return false;
}

bool nav_linefollow() {
	if (!linefollow_start(60))
		return false;
	linefollow_waitDone();
	return true;
}

