#include "competition/navdeploy.h"
#include "competition/nav.h"
#include "control/drive.h"
#include "control/deploy.h"
#include "control/linefollow.h"
#include "debug/debug.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdio.h>

void navdeploy_lap() {
	for (int i=0; i<2; i++) {
		if (!navdeploy_loopback()) {
			printf_P(PSTR("Failed loopback\n"));
			return;
		}
		if (i == 1) {
			drive_lturnDeg(60, 15);
		}
		navdeploy_deploy();
		if (!navdeploy_aroundBox()) {
			printf_P(PSTR("Failed aroundbox\n"));
			return;
		}
		if (!navdeploy_middle()) {
			printf_P(PSTR("Failed middle\n"));
			return;
		}
		navdeploy_deploy();
		if (!navdeploy_aroundBox()) {
			printf("Failed aroundbox\n");
			return;
		}
		navdeploy_end();
	}
}

void navdeploy_deploy() {		// Run when encountering a box
	deploy_waitDone();			// prime sensor on deployer
	_delay_ms(500);
	drive_fdDist(20, 15);		// approach box
	deploy_out(true);			// release sensor
	_delay_ms(500);
	drive_bkDist(1, .25);
	_delay_ms(500);

	drive_bk(4);				// drive backwards from box
	_delay_ms(200);
	deploy_off();
	linefollow_waitLine();		// until the line is seen

	deploy_start();				// start priming next sensor on deployer
}

bool navdeploy_aroundBox() {// Run immediately after dropping one sensor off, to navigate around box to next box or loopback
	drive_bkDist(30, 10);		// in front of first box
	drive_lturnDeg(60, 55);
	drive_fd(60);
	drive_waitDist(10);
	bool noturn=false;
	if (!nav_waitLineDist(7, 7, 25)) {	// missed corner completely
		drive_stop();
		printf_P(PSTR("Missed first line!"));
		drive_fdDist(60, 10);
		drive_rturnDeg(60, 95);			// turn to face line (hopefully)
		drive_fd(60);
		if (!nav_waitLineDist(0, 3, 30)) {
			printf_P(PSTR("TODO: Timed out looking for line"));
			drive_stop();
			return false;
		} else {
			drive_cStop();
			drive_lturnDeg(60, 20);
			noturn = true;
		}
	} else {

		drive_waitDist(6);

		if (linefollow_getLine(0, 7)) { // if we still see the line
			if (!nav_waitLineDist(7, 7, 25)) { // do a long timeout
				if (!linefollow_getLine(2, 6)) {
					drive_stop();
					printf_P(PSTR("TODO Missed second line!"));
					return false;
				} else {
					printf_P(PSTR("Missed second line, but still able to follow"));
					noturn = true;
				}
			}
		} else {
			printf_P(PSTR("Nicked corner!\n"));
			drive_fdDist(60, 10);
			drive_rturnDeg(60, 95);			// turn to face line (hopefully)
			drive_fd(60);
			if (!nav_waitLineDist(0, 3, 30)) {
				printf_P(PSTR("TODO: Timed out looking for line"));
				drive_stop();
				return false;
			} else {
				drive_cStop();
				drive_lturnDeg(60, 20);
				noturn = true;
			}
		}
	}
	drive_cStop();
	nav_pause();
	
	if (!noturn) {
		drive_rturnDeg(60, 40);
		nav_pause();
	}

	if (!nav_linefollow(.4))
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

bool navdeploy_middle() {	// Run when navigating the space between the two boxes on either row of the course
	drive_rturnDeg(50, 43);		// on back left corner of first box
	drive_fd(60);
	drive_waitDist(12);
	linefollow_waitLine(3, 4);	// on middle line connecting two boxes
	drive_waitDist(2);
	drive_cStop();
	drive_lturnDeg(50, 60);		// on middle line facing second box
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

void navdeploy_end() {		// Run after second box has been navigated around on one side to get to loopback position
	drive_rturnDeg(50, 30);		// back left corner, second box
	drive_fd(60);
	drive_waitDist(10);
	linefollow_waitLine(3, 4);	// crossing first loopback line
	drive_waitDist(10);
	linefollow_waitLine(6, 7);
	drive_cStop();				// on main loopback line
	drive_rturnDeg(60, 45);
}
