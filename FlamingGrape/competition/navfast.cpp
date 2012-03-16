#include "competition/navfast.h"
#include "competition/nav.h"
#include "competition/sensorcomms.h"
#include "competition/sensordecision.h"
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
	uint8_t halflap=0;
	for (int i=0; i<2; i++, halflap++) {
		printf_P(PSTR("Entering loopback\n"));
		if (!navfast_loopback(halflap)) {
			printf_P(PSTR("Failed loopback\n"));
			return;
		}
		
		bool right = sensordecision_isRight();
		printf_P(PSTR("Entering leftright\n"));
		if (!navfast_leftright(right, halflap)) {
			printf_P(PSTR("Failed leftright\n"));
			return;
		}
		
		bool cross = sensordecision_isRight() != right;
		if (cross) {
			printf_P(PSTR("Entering cross\n"));
			if (!navfast_cross(right)) {
				printf_P(PSTR("Failed cross\n"));
				return;
			}
			
			right = !right;// we're on the opposite side of the board now
		} else {
			nav_pause();
			turn(60, 15, right);
			drive_cStop();
		}
		
		printf_P(PSTR("Entering jump\n"));
		if (!navfast_jump(right)) {
			printf_P(PSTR("Failed jump\n"));
			return;
		}
		
		printf_P(PSTR("Entering end\n"));
		navfast_end(right);
	}
}

bool navfast_loopback(uint8_t hlap) {
	if (!nav_linefollowTurns(1, 0.5))
		return false;
	_delay_ms(100);
	
	sensordecision_prepare((hlap & 0x01) == 0 ? 0 : 2);
		
	if (!nav_linefollowDist(35))
		return false;
		
	drive_cStop();
	nav_pause();
	return true;
}

bool navfast_leftright(bool right, uint8_t halflap) {
	if (right)
		drive_rturnDeg(60, 45);
	else
		drive_lturnDeg(60, 45);
	nav_pause();
	
	uint8_t sensor = right ? 0 : 7;
	drive_fd(60);				
	drive_waitDist(15);
	if (!nav_waitLineDist(sensor, sensor, 27)) {
		drive_stop();
		printf_P(PSTR("TODO Missed first line!"));
		return false;
	}
	
	drive_waitDist(6);
	
	bool noturn=false;
	if (linefollow_getLine(0, 7)) { // if we still see the line
		if (!nav_waitLineDist(sensor, sensor, 25)) { // do a long timeout
			if (!linefollow_getLine(0, 7)) {
				drive_stop();
				printf_P(PSTR("TODO Missed second line!"));
				return false;
			} else {
				printf_P(PSTR("Missed second line, but still able to follow"));
				noturn = true;
			}
		} else {
			drive_cStop();
			turn(60, 40, !right);
			nav_pause();
		}
	} else {
		printf_P(PSTR("Nicked corner!\n"));
		drive_cStop();
		turn(60, 60, !right);
		drive_fd(60);
		if (!nav_waitLineDist(3, 4, 20)) {
			drive_stop();
			printf_P(PSTR("Failed to find line after nicked corner!\n"));
			return false;
		}
		
		noturn = true;
	}
	
	drive_cStop();
	nav_pause();
	
	sensordecision_prepare((halflap & 0x01 == 0) ? 1 : 3);
	
	if (!nav_linefollow(right ? -.4 : .4))
		return false;
		
	drive_cStop();
	nav_pause();
	return true;
}

bool navfast_cross(bool right) {
	turn(60, 70, !right);
	nav_pause();
	
	if (!nav_linefollow(right ? -.6 : .6)) // follow to intersection
		return false;
	nav_pause();
	
	drive_fdDist(60, 5, DM_BANG); // go past intersection
	nav_pause();
	
	if (!nav_linefollow(right ? -.6 : .6)) // follow to turn	
		return false;
	drive_cStop();
	nav_pause();
	
	turn(60, 95, right);
	nav_pause();
	return true;
}

// TODO better corner finder
// handle line visible always (hit box, veered)
// handle nicked corner

bool navfast_jump(bool right) {
	drive_fd(60);
	drive_waitDist(4);
	bool outside = false;
	bool inside = false;
	
	if (!nav_waitLineDist(0, 7, 33)) {
		outside = true;
	} else {
		_delay_ms(30);
		LineFollowResults results = linefollow_readSensor();
		
		if (results.thresh_count >= 5) {
			printf("Maybe intersection\n");
			drive_waitDist(4);
			if (!linefollow_getLine(2, 5))
				inside = true;
		} else if ((right && results.center < -.9) || (!right && results.center > .9) || (results.thresh_count == 0)) {
			outside = true;
		} else  {
			drive_waitDist(3);
		}
	}
		
	if (outside) {
		drive_cStop();
		printf_P(PSTR("Overshot outside\n"));// TODO occured when too far back from line, turned into the corner, went straight at it, read as a turn, jumped to end
		
		turn(60, 55, !right);
		
		drive_fd(60);
		if (!nav_waitLineDist(3, 4, 35)) {
			drive_stop();
			printf_P(PSTR("Couldn't find line after overshoot outside!\n"));
			return false;
		}
		drive_cStop();
		
		turn(60, 30, right);
		
		if (!nav_linefollow(right ? -.4 : .4)) {
			printf_P(PSTR("Line disappeared after overshoot outside!\n"));
			return false;
		}
	} else if (inside) { // inside
		drive_cStop();
		printf_P(PSTR("Going to hit box!\n"));
		
		turn(60, 45, right);
		drive_fd(60);
		if (!nav_waitLineDist(2, 5, 20)) {
			drive_stop();
			printf_P(PSTR("Couldn't find line from the inside\n"));
			return false;
		}
		
		if (!nav_linefollow(right ? -.4 : .4)) {
			drive_stop();
			printf_P(PSTR("Line disappeared after inside\n"));
			return false;
		}
	} else { // hit line correctly
		if (!nav_linefollow(right ? -.4 : .4)) {
			drive_stop();
			printf_P(PSTR("Line disappeared, turning right!\n"));	// TODO Hit this case when it looked like it should have been running great
			
			turn(60, 10, right);
			if (!nav_linefollow(right ? -.4 : .4)) {
				printf_P(PSTR("Line still gone, bailing\n"));
				return false;
			}
		}
	}
	
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
		drive_waitDist(3);			// Go forward a few more centimeters to center on line when turned
		drive_cStop();
		drive_rturnDeg(60, 90);		// Turn to face direction of loopback
	} else {					// If we're on the back left corner
		drive_rturnDeg(60, 30);		// Turn to intersect loopback line
		drive_fd(60);				// Start going towards the loopback line
		drive_waitDist(10);			// Wait a little before looking for the line to escape line currently on
		linefollow_waitLine(3, 4);	// Wait for the middle sensors to see a line, this is our first crossover
		drive_waitDist(10);			// Keep going to get off that line
		linefollow_waitLine(6, 7);	// Wait until the right side of the sensor is triggered (this will be loopback line to follow)
		drive_cStop();
		drive_rturnDeg(60, 45);		// Turn to face direction on new loopback line
	}
}
