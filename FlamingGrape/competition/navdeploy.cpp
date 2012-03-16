#include "competition/navdeploy.h"
#include "competition/nav.h"
#include "competition/sensordecision.h"
#include "control/drive.h"
#include "control/deploy.h"
#include "control/linefollow.h"
#include "debug/debug.h"
#include "hw/tick.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdio.h>

void navdeploy_lap() {
	uint32_t count = tick_getCount();
	uint16_t box = 0;
	for (int i=0; i<2; i++) {
		if (!navdeploy_loopback()) {
			printf_P(PSTR("Failed loopback\n"));
			return;
		}
		if (i == 0){
			drive_lturnDeg(60, 5);
			box = 0;
		} else {
			box = 2;
		}
		navdeploy_deploy(box);
		bool right = sensordecision_isRight();
		if (!navdeploy_aroundBox(right)) {
			printf_P(PSTR("Failed aroundbox\n"));
			return;
		}
		if (!navdeploy_middle(right)) {
			printf_P(PSTR("Failed middle\n"));
			return;
		}
		navdeploy_deploy(box + 1, i == 1);
		right = sensordecision_isRight();
		if (!navdeploy_aroundBox(right)) {
			printf("Failed aroundbox\n");
			return;
		}
		navdeploy_end(right);
	}
}

void navdeploy_deploy(int box, bool lastbox) {		// Run when encountering a box
	deploy_waitDone();			// prime sensor on deployer
	_delay_ms(500);
	drive_fd(20);		// approach box
	drive_waitDist(15);
	deploy_out(true);			// release sensor
	if (box == 0) {
		_delay_ms(3000);
	} else {
		_delay_ms(3500);
	}
	drive_stop();
	drive_bkDist(1, .25);
	_delay_ms(2500);

	drive_bk(4);				// drive backwards from box
	_delay_ms(200);
	deploy_off();
	linefollow_waitLine();		// until the line is seen

	if (!lastbox)
		deploy_start();				// start priming next sensor on deployer
	sensordecision_prepare(0);		// TODO Replace 0 with box!!
	sensordecision_wait();
}

bool navdeploy_aroundBox(bool same) {// Run immediately after dropping one sensor off, to navigate around box to next box or loopback
	drive_bkDist(30, 10);		// in front of first box
	drive_turn(60, 50, same);
	drive_fd(60);
	drive_waitDist(10);
	bool noturn=false;
	uint16_t sensor = !same ? 7 : 0;
	if (!nav_waitLineDist(sensor, sensor, 25)) {	// missed corner completely
		drive_stop();
		printf_P(PSTR("Missed first line!\n"));
		drive_fdDist(60, 10);
		drive_turn(60, 105, !same);			// turn to face line (hopefully)
		drive_fd(60);
		uint16_t sensorl = !same ? 0 : 4;
		uint16_t sensorr = !same ? 3 : 7;
		if (!nav_waitLineDist(sensorl, sensorr, 30)) {
			printf_P(PSTR("TODO: Timed out looking for line\n"));
			drive_stop();
			return false;
		} else {
			drive_cStop();
			drive_turn(60, 20, same);
			noturn = true;
		}
	} else {

		drive_waitDist(6);

		if (linefollow_getLine(0, 7)) { // if we still see the line
			if (!nav_waitLineDist(sensor, sensor, 25)) { // do a long timeout
				uint16_t sensorl = !same ? 2 : 1;
				uint16_t sensorr = !same ? 6 : 5;
				if (!linefollow_getLine(sensorl, sensorr)) {
					drive_stop();
					printf_P(PSTR("TODO Missed second line!\n"));
					return false;
				} else {
					printf_P(PSTR("Missed second line, but still able to follow\n"));
					noturn = true;
				}
			}
		} else {
			printf_P(PSTR("Nicked corner!\n"));
			drive_fdDist(60, 10);
			drive_turn(60, 95, !same);			// turn to face line (hopefully)
			drive_fd(60);
			uint16_t sensorl = !same ? 0 : 4;
			uint16_t sensorr = !same ? 3 : 7;
			if (!nav_waitLineDist(sensorl, sensorr, 30)) {
				printf_P(PSTR("TODO: Timed out looking for line\n"));
				drive_stop();
				return false;
			} else {
				drive_cStop();
				drive_turn(60, 20, same);
				noturn = true;
			}
		}
	}
	drive_cStop();
	nav_pause();

	if (!noturn) {
		drive_turn(60, 40, !same);
		nav_pause();
	}

	float offset = !same ? 0.4 : -0.4;
	if (!nav_linefollow(offset))
		return false;

	drive_cStop();
	nav_pause();
	return true;
}

bool navdeploy_loopback() {	// Run when leaving one row of boxes to loop back to the other row of boxes
	if (!nav_linefollowIntersection())	// linefollow until intersection before first box is reached
		return false;
	return true;
}

bool navdeploy_middle(bool same) {	// Run when navigating the space between the two boxes on either row of the course
	drive_turn(50, 50, !same);		// on back left corner of first box
	drive_fd(60);
	drive_waitDist(12);
	linefollow_waitLine(3, 4);	// on middle line connecting two boxes
	drive_waitDist(2);
	drive_cStop();
	drive_turn(50, 60, same);		// on middle line facing second box
	if (!linefollow_start(60)) {
		printf_P(PSTR("Overshot middle line!\n"));
		drive_bkDist(60, 10);
		if (!linefollow_start(60)) {
			printf_P(PSTR("Still couldn't find middle line!\n"));
			return false;
		}
	}
	linefollow_waitDone();
	drive_cStop();				// on intersection before second box
	drive_bkDist(40, 14);
	if (!linefollow_start(60))
		return false;
	linefollow_waitDone();		// on intersection after re-align
	drive_cStop();
	return true;
}
// TODO Make nav_end shared in deploy/fast
void navdeploy_end(bool same) {		// Run after second box has been navigated around on one side to get to loopback position
	if (same) {				// If we're on the back right corner
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
