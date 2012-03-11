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

static void turn(float vel, float deg, bool right) {
	if (right)
		drive_rturnDeg(vel, deg);
	else
		drive_lturnDeg(vel, deg);
}

void navfast_lap() {
	for (int i=0; i<2; i++) {
		printf_P(PSTR("Entering loopback\n"));
		if (!navfast_loopback()) {
			printf_P(PSTR("Failed loopback\n"));
			return;
		}
		
		uint32_t val = tick_getCount();
		bool right=((val&0x01) != 0);
		//bool cross=((val&0x02) != 0);
		bool cross=false;
		
		printf_P(PSTR("Entering leftright\n"));
		if (!navfast_leftright(right)) {
			printf_P(PSTR("Failed leftright\n"));
			return;
		}
		
		if (cross) {
			printf_P(PSTR("Entering cross\n"));
			if (!navfast_cross(right)) {
				printf_P(PSTR("Failed cross\n"));
				return;
			}
			
			right = !right;// we're on the opposite side of the board now
		} else {
			printf_P(PSTR("Entering jump\n"));
			if (!navfast_jump(right)) {
				printf_P(PSTR("Failed jump\n"));
				return;
			}
		}
		
		printf_P(PSTR("Entering end\n"));
		navfast_end(right);
	}
}

bool navfast_loopback() {
	if (!nav_linefollowTurns(1, 0.5))
		return false;
	_delay_ms(50);
	if (!nav_linefollowDist(35))
		return false;
		
	drive_stop();
	_delay_ms(300);
	nav_pause();
	return true;
}

bool navfast_leftright(bool right) {
	if (right)
		drive_rturnDeg(60, 38);
	else
		drive_lturnDeg(60, 45);
	nav_pause();
	
	uint8_t sensor = right ? 0 : 7;
	drive_fd(60);				
	drive_waitDist(15);
	if (!nav_waitLineDist(sensor, sensor, 20)) {
		drive_stop();
		printf_P(PSTR("TODO Missed first line!"));
		return false;
	}
	
	drive_waitDist(4);
	
	if (linefollow_getLine(0, 7)) { // if we still see the line
		if (!nav_waitLineDist(sensor, sensor, 25)) { // do a long timeout
			if (!linefollow_getLine(2, 6)) {
				drive_stop();
				printf_P(PSTR("TODO Missed second line!"));
				return false;
			} else {
				printf_P(PSTR("Missed second line, but still able to follow"));
			}
		}
	} else {
		printf_P(PSTR("Nicked corner!\n"));
	}
	
	drive_stop();
	_delay_ms(300);
	nav_pause();
	
	turn(60, 40, !right);
	nav_pause();
	
	if (!nav_linefollow(right ? -.4 : .4))
		return false;
		
	drive_stop();
	_delay_ms(300);
	nav_pause();
	return true;
}

bool navfast_cross(bool right) {
	debug_setLED(OTHERYELLOW_LED, true);
	
	turn(60, 60, !right);
	nav_pause();
	
	if (!nav_linefollow(right ? -.6 : .6)) // follow to intersection
		return false;
	nav_pause();
	
	drive_fdDist(60, 5, DM_BANG); // go past intersection
	nav_pause();
	
	if (!nav_linefollow(right ? -.6 : .6)) // follow to turn	
		return false;
	drive_stop();
	_delay_ms(300);
	nav_pause();
	
	turn(60, 95, right);
	nav_pause();

	drive_fd(60);
	if (!nav_waitLineDist(0, 7, 25)) {
		drive_stop();
		_delay_ms(300);
		turn(60, 45, right);
		drive_fd(60);
		if (!nav_waitLineDist(0, 7, 40)) {
			drive_stop();
			printf_P(PSTR("Wat do?? Double cross fail\n"));
			return false;
		}
		drive_waitDist(5);
		drive_stop();
		_delay_ms(300);
		
		turn(60, 30, !right);
		drive_stop();
		nav_pause();
	} else {
		drive_waitDist(3);
		drive_stop();
		nav_pause();
	}
	
	if (!nav_linefollow(right ? .4 : -.4)) {
		printf(PSTR("Line gone??\n"));
		return false;
	}
		
	drive_stop();
	_delay_ms(300);
	
	debug_setLED(OTHERYELLOW_LED, false);
	return true;
}

// TODO better corner finder
// handle line visible always (hit box, veered)
// handle nicked corner

static bool jumpFix(bool right);

bool navfast_jump(bool right) {
	nav_pause();
	turn(60, 15, right);
	
	bool jumpfix=false;
	nav_pause();
	drive_fd(60);
	drive_waitDist(4);
	if (!nav_waitLineDist(0, 7, 25)) {
		printf_P(PSTR("A"));
		jumpfix = true;
		if (!jumpFix(right))
			return false;
	} else {
		drive_waitDist(3);
	}
	
	DriveDist dd;
	drive_initDist(dd);
	
	if (!nav_linefollow(right ? -.4 : .4)) {
		printf_P(PSTR("B"));
		jumpfix = true;
		if (!jumpFix(right))
			return false;
		if (!nav_linefollow(right ? -.4 : .4)) {
			printf_P(PSTR("Line disappeared after nicked-corner jumpfix\n"));
			return false;
		}
	}
	if (linefollow_getLastFeature() == FEATURE_INTERSECTION || (!jumpfix && drive_getDist(dd) < 10)) { // TODO not really good enough, use future drive measurement stuff
		printf("Intersection/shortline fix!\n");
		turn(60, 45, right);
		nav_pause();
		
		drive_stop();
		_delay_ms(300);
		
		drive_fd(60);
		uint8_t sensor = right ? 7 : 0;
		if (!nav_waitLineDist(sensor, sensor, 20)) {
			printf_P(PSTR("No line after intersection/shortline fix, turning back to find line"));
			
			turn(60, 65, !right);
			drive_stop();
			_delay_ms(300);
			
			drive_fd(60);
			if (!nav_waitLineDist(0, 7, 20) || !nav_linefollow(right ? -.4 : .4)) {
				drive_stop();
				printf_P(PSTR("Giving up on intersection/shortline\n"));
				return false;
			}
		} else {
			if (!nav_linefollow(right ? -.4 : .4)) {
				printf_P(PSTR("Line disappeared after intersection/shortline fix\n"));
				return false;
			}
		}
	}
		
	linefollow_waitDone();
	drive_stop();
	_delay_ms(300);
	return true;
}

static bool jumpFix(bool right) {
	printf_P(PSTR("jumpFix!\n"));
	
	drive_stop();
	_delay_ms(300);
	turn(60, 45, !right);
	drive_fd(60);
	if (!nav_waitLineDist(0, 7, 40)) {
		drive_stop();
		printf_P(PSTR("Wat do? Jump fix fail\n"));
		return false;
	}
	drive_waitDist(5);
	drive_stop();
	_delay_ms(300);
	
	turn(60, 20, right);
	drive_stop();
	return true;
}

// TODO tonight
// FIXMEs
// slew rate limiting
// waitLineDist
// better turn detection

void navfast_end(bool right) {	// Run after a row of boxes to return to main line for loopback
	_delay_ms(4000);
	if (right) {				// If we're on the back right corner
		drive_fd(60);				// Start going straight forward to intersect loopback line
		drive_waitDist(10);			// Wait a little before looking for the line to escape line currently on
		linefollow_waitLine();		// Drive until we intersect loopback line
		drive_stop();
		_delay_ms(300);
		drive_rturnDeg(60, 80);		// Turn to face direction of loopback
	} else {					// If we're on the back left corner
		drive_rturnDeg(60, 30);		// Turn to intersect loopback line
		drive_fd(60);				// Start going towards the loopback line
		drive_waitDist(10);			// Wait a little before looking for the line to escape line currently on
		linefollow_waitLine(3, 4);	// Wait for the middle sensors to see a line, this is our first crossover
		drive_waitDist(10);			// Keep going to get off that line
		linefollow_waitLine(6, 7);	// Wait until the right side of the sensor is triggered (this will be loopback line to follow)
		drive_stop();
		_delay_ms(300);
		drive_rturnDeg(60, 45);		// Turn to face direction on new loopback line
	}
}
