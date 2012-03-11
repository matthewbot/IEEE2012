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
	if (!nav_linefollowTurns(2, 0.5))
		return false;
	if (!nav_linefollowDist(35))
		return false;
		
	drive_stop();
	_delay_ms(300);
	nav_pause();
	return true;
}

bool navfast_leftright(bool right) {
	if (right)
		drive_rturnDeg(60, 32);
	else
		drive_lturnDeg(60, 45);
	nav_pause();
	
	uint8_t sensor = right ? 0 : 7;
	drive_fd(60);				
	drive_waitDist(15);
	linefollow_waitLine(sensor, sensor);
	drive_waitDist(1); // FIXME
	linefollow_waitLine(sensor, sensor, true); // wait till off the line
	linefollow_waitLine(sensor, sensor);
	drive_stop();
	_delay_ms(300);
	nav_pause();
	
	turn(60, 40, !right);
	nav_pause();
	
	if (!linefollow_start(60, right ? -.4 : .4))
		return false;
		
	linefollow_waitDone();
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
	
	turn(60, 100, right);
	nav_pause();

	drive_fd(60);
	linefollow_waitLine(0, 7);
	drive_waitDist(5); // FIXME
	drive_stop();
	nav_pause();
	
	if (!linefollow_start(60, right ? .4 : -.4)) {
		printf("Special fix\n");
		debug_setLED(GREEN_LED, true);
		turn(60, 30, right);
		nav_pause();
		
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
	nav_pause();
	turn(60, 10, right);
	
	nav_pause();
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

void navfast_end(bool right) { // FIXME: skip last corner
	turn(60, 40, !right);
		
	drive_fd(60);
	drive_waitDist(15);
	linefollow_waitLine();
	drive_waitDist(5);
	drive_stop();
	_delay_ms(300);
	
	turn(60, 40, right);
}
