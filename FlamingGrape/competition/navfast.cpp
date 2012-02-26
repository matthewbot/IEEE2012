#include "competition/navfast.h"
#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "hw/adc.h"
#include "hw/tick.h"
#include "debug/debug.h"
#include <stdio.h>
#include <util/delay.h>

bool navfast_loopback() {
	if (!nav_linefollowTurns(2))
		return false;
	
	if (!linefollow_start(60))
		return false;
	_delay_ms(100);
	
	while (!linefollow_isDone()) {
		float reading = adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE);
		if (reading < 35) {
			linefollow_stop();
			drive_stop();
			return true;
		}
	}
			
	drive_stop();
	return false;
}

bool navfast_leftright(bool right) {
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
	
	if (!linefollow_start(60))
		return false;
		
	linefollow_waitDone();
	return true;
}
