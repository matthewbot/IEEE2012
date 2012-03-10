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
	static bool cross;
	
	if (!right) {
		right = true;
	} else {
		right = false;
		cross = !cross;
	}
	
	if (!navfast_leftright(right)) {
		printf_P(PSTR("Failed leftright\n"));
		return;
	}
	
	if (cross) {
		if (!navfast_cross(right)) {
			printf_P(PSTR("Failed cross\n"));
			return;
		}
		
		navfast_end(!right); // we're on the opposite side of the board now
	} else {
		if (!navfast_jump(right)) {
			printf_P(PSTR("Failed jump\n"));
			return;
		}
		
		navfast_end(right);
	}
}

bool navfast_loopback() {
	printf("A\n");
	if (!nav_linefollowTurns(2))
		return false;
	
	printf("B\n");
	if (!nav_linefollowRange(35))
		return false;
			
	return true;
}

static void turn(float vel, float deg, bool right) {
	if (right)
		drive_rturnDeg(vel, deg);
	else
		drive_lturnDeg(vel, deg);
}

bool navfast_leftright(bool right) {
	turn(60, 55, right);

	uint8_t sensor = right ? 0 : 7;		
	drive_fd(60);				
	drive_waitDist(12);
	linefollow_waitLine(sensor, sensor);
	drive_waitDist(2);
	linefollow_waitLine(sensor, sensor);
	drive_stop();				
	_delay_ms(300);				
	
	turn(60, 45, !right);
	
	if (!linefollow_start(60, right ? -.4 : .4))
		return false;
		
	linefollow_waitDone();
	return true;
}

bool navfast_cross(bool right) {
	drive_fdDist(60, 10);
	turn(60, 90, !right);
	
	drive_fd(60);
	linefollow_waitLine();
	drive_fdDist(60, 10);
	
	turn(60, 90, right);
	drive_fd(60);
	linefollow_waitLine();
	drive_waitDist(2);
	
	if (!linefollow_start(60, right ? .4 : -.4))
		return false;
		
	linefollow_waitDone();
	return true;
}

bool navfast_jump(bool right) {
	drive_fd(60, DM_BANG); // Bang so we don't re-accelerate from 0 after the previous command
	drive_waitDist(4);
	linefollow_waitLine(0, 7);
	drive_waitDist(2);
	
	if (!linefollow_start(60, right ? -.4 : .4))
		return false;
		
	linefollow_waitDone();
	return true;
}

void navfast_end(bool right) {
	turn(50, 40, !right);
		
	drive_fd(60);
	drive_waitDist(10);
	linefollow_waitLine(0, 3);
	drive_stop();
	_delay_ms(300);
	
	turn(50, 60, right);
}
