#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "hw/adc.h"
#include "hw/tick.h"
#include "debug/debug.h"
#include <stdio.h>
#include <util/delay.h>


bool nav_linefollowIntersection() {
	while (true) {
		linefollow_start(60);
		linefollow_waitDone();
		
		if (linefollow_getLastFeature() == FEATURE_INTERSECTION)
			return true;
		else if (linefollow_getLastTurn() == TURN_LEFT)
			drive_lturn_deg(50, 80);
		else if (linefollow_getLastTurn() == TURN_RIGHT)
			drive_rturn_deg(50, 80);
		else
			return false;
	}
}

bool nav_linefollowTurns(int turncount) {
	while (true) {
		linefollow_start(60);
		linefollow_waitDone();
		
		if (linefollow_getLastTurn() == TURN_LEFT) {
			if (--turncount <= 0)
				return true;
			drive_lturn_deg(50, 80);
		} else if (linefollow_getLastTurn() == TURN_RIGHT) {
			if (--turncount <= 0)
				return true;
			drive_rturn_deg(50, 80);
		} else {
			return false;
		}
	}	
}
