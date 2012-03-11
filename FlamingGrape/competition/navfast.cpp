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
		bool cross = true;
		
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
		drive_rturnDeg(60, 34);
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
	
	drive_waitDist(1);
	linefollow_waitLine(sensor, sensor, true); // wait till off the line
	
	if (!nav_waitLineDist(sensor, sensor, 25)) {
		drive_stop();
		printf_P(PSTR("TODO Missed second line!"));
		return false;
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
	
	turn(60, 100, right);
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
		drive_waitDist(1);
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

bool navfast_jump(bool right) {
	nav_pause();
	turn(60, 15, right);
	
	nav_pause();
	drive_fd(60);
	drive_waitDist(4);
	if (!nav_waitLineDist(0, 7, 25)) {
		drive_stop();
		_delay_ms(300);
		turn(60, 45, !right);
		drive_fd(60);
		if (!nav_waitLineDist(0, 7, 40)) {
			drive_stop();
			printf_P(PSTR("Wat do?? Double jump fail\n"));
			return false;
		}
		drive_waitDist(5);
		drive_stop();
		_delay_ms(300);
		
		turn(60, 20, right);
		drive_stop();
	} else {
		drive_waitDist(3);
	}
	
	if (!nav_linefollow(right ? -.4 : .4))
		return false;
	if (linefollow_getLastFeature() == FEATURE_INTERSECTION) { // TODO not really good enough, use future drive measurement stuff
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
