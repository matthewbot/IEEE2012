#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "hw/adc.h"
#include "hw/tick.h"
#include "debug/debug.h"
#include <stdio.h>

#include <util/delay.h>

bool nav_loopback() {
	uint8_t turncount = 0;
	
	while (true) {
		linefollow_start(60);
		
		uint8_t ctr=0;
		while (!linefollow_isDone()) {
			float reading = adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE);
			if (ctr > 50) {
				if (turncount >= 2 && reading < 35) {
					linefollow_stop();
					drive_stop();
					return true;
				}
			} else {
				ctr++;
			}
			tick_wait();
		}
		
		if (linefollow_getLastFeature() == FEATURE_NOLINE) {
			if (linefollow_getLastTurn() == TURN_LEFT) {
				drive_lturn_deg(80, 50);
				turncount++;
			} else if (linefollow_getLastTurn() == TURN_RIGHT) {
				drive_rturn_deg(80, 50);
				turncount++;
			} else {
				break;
			}
		} else {
			break;
		}
	}
	
	drive_stop();
	return false;
}

bool nav_leftright(bool right) {
	if (right) {
		drive_rturn_deg(30, 50);
	} else {
		drive_lturn_deg(20, 50);
	}
	
	drive_fd_dist(60, 30);
	
	if (right) {
		drive_lturn_deg(30, 50);
	} else {
		drive_rturn_deg(30, 50);
	}
	
	linefollow_start(60);
	linefollow_waitDone();
	return true;
}
