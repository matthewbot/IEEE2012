#include "competition/navfast.h"
#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "hw/adc.h"
#include "hw/tick.h"
#include "debug/debug.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>

void navfast_lap() {
	if (!navfast_loopback()) {
		printf_P(PSTR("Failed loopback\n"));
		return;
	}
	
	static bool right;
	right = !right;
	if (!navfast_leftright(right)) {
		printf_P(PSTR("Failed leftright\n"));
		return;
	}
}

bool navfast_loopback() {
	if (!nav_linefollowTurns(2))
		return false;
	
	if (!nav_linefollowRange(35))
		return false;
			
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
