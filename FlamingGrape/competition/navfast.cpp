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

static void pause() {
	//_delay_ms(1000);
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
		bool cross=((val&0x02) != 0);
		
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
	if (!nav_linefollowDist(35))
		return false;
		
	drive_stop();
	_delay_ms(300);
	return true;
}

static void turn(float vel, float deg, bool right) {
	if (right)
		drive_rturnDeg(vel, deg);
	else
		drive_lturnDeg(vel, deg);
}

bool navfast_leftright(bool right) {
	pause();
	if (right)
		drive_rturnDeg(60, 32);
	else
		drive_lturnDeg(60, 45);
	pause();
	
	uint8_t sensor = right ? 0 : 7;
	drive_fd(60);				
	drive_waitDist(15);
	linefollow_waitLine(sensor, sensor);
	drive_waitDist(1); // FIXME
	linefollow_waitLine(sensor, sensor, true); // wait till off the line
	linefollow_waitLine(sensor, sensor);
	drive_stop();
	_delay_ms(300);
	
	pause();
	turn(60, 40, !right);
	pause();
	
	
	if (!linefollow_start(60, right ? -.4 : .4))
		return false;
		
	linefollow_waitDone();
	drive_stop();
	_delay_ms(300);
	return true;
}
/*
bool navfast_cross(bool right) {
	pause();
	if (right)
		drive_rturnDeg(60, 5);
	else
		drive_lturnDeg(60, 15);
	pause();
	
	pause();
	drive_fdDist(60, 15);
	pause();
	if (right)
		drive_lturnDeg(60, 95);
	else
		drive_rturnDeg(60, 85);
	
	drive_fd(60);
	linefollow_waitLine();
	drive_stop();
	pause();
	drive_fdDist(60, 25);
	pause();
	
	if (right)
		drive_rturnDeg(60, 75);
	else
		drive_lturnDeg(60, 85);

	pause();
	drive_fd(60);
	linefollow_waitLine(0, 7);
	drive_waitDist(3);
	
	if (!linefollow_start(60, right ? .4 : -.4))
		return false;
		
	linefollow_waitDone();
	return true;
}
*/

bool navfast_cross(bool right) {
	debug_setLED(OTHERYELLOW_LED, true);
	
	pause();
	turn(60, 60, !right);
	pause();
	
	/*if (!nav_linefollowDist(23))
		return false;
	
	debug_setLED(GREEN_LED, true);
	drive_fdDist(60, 5, DM_BANG);
	debug_setLED(GREEN_LED, false);*/
	
	if (!nav_linefollow(right ? -.6 : .6)) // follow to intersection
		return false;
	drive_fdDist(60, 5, DM_BANG); // go past intersection
	if (!nav_linefollow(right ? -.6 : .6)) // follow to turn	
		return false;
	
	drive_stop();
	_delay_ms(300);
	
	pause();
	turn(60, 100, right);

	pause();
	drive_fd(60);
	linefollow_waitLine(0, 7);
	drive_waitDist(5); // FIXME
	drive_stop();
	
	pause();
	if (!linefollow_start(60, right ? .4 : -.4)) {
		printf("Special fix\n");
		debug_setLED(GREEN_LED, true);
		turn(60, 30, right);
		drive_fd(20);
		linefollow_waitLine(0, 7);
		drive_waitDist(1);
		if (!linefollow_start(60, right ? .4 : -.4))
			return false;
		debug_setLED(GREEN_LED, false);
	}
		
	linefollow_waitDone();
	drive_stop();
	_delay_ms(300);
	
	debug_setLED(OTHERYELLOW_LED, false);
	return true;
}

bool navfast_jump(bool right) {
	pause();
	turn(60, 10, right);
	
	drive_fd(60);
	drive_waitDist(4);
	linefollow_waitLine(0, 7);
	drive_waitDist(2);
	
	if (!nav_linefollow(right ? -.4 : .4))
		return false;
	if (linefollow_getLastFeature() == FEATURE_INTERSECTION) {
		printf("Intersection fix!\n");
		turn(60, 15, right);
		drive_fdDist(60, 5, DM_BANG);
		if (!nav_linefollow(right ? -.4 : .4))
			return false;
	}
		
	linefollow_waitDone();
	drive_stop();
	_delay_ms(300);
	return true;
}

// TODO tonight
// FIXMEs
// slew rate limiting
// waitLineDist
// better turn detection

void navfast_end(bool right) {	// Run after a row of boxes to return to main line for loopback
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
