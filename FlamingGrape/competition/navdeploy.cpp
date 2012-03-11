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
	_delay_ms(2000);
	
	for (int i = 0; i < 2; i++) {	// Do 2 humps
		drive_bkDist(20, 5);
		deploy_off();
		drive_fdDist(20, 7);
	}
	
	drive_bk(4);				// drive backwards from box
	linefollow_waitLine();		// until the line is seen
	
	deploy_start();				// start priming next sensor on deployer
}

bool navdeploy_aroundBox() {	// Run immediately after dropping one sensor off, to navigate around box to next box or loopback
	drive_bkDist(30, 10);		// backup to give room when cutting corner around box
	drive_lturnDeg(60, 55);	// turn left to go around left front corner of box
	drive_fd(60);				// drive forward to the left side of the box
	drive_waitDist(10);
	linefollow_waitLine(7, 7);	// notice when the first line is crossed as we cut the corner of the line surrounding the box
	drive_waitDist(10);
	linefollow_waitLine(7, 7);	// when the second line is seen of the corner of the line surrounding the box
	drive_cStop();				// wait for bouncing to stop
	drive_rturnDeg(60, 45);	// turn right to face parralel to course sitting next to box
	
	if (!linefollow_start(60, .4))	// Linefollow
		return false;
	linefollow_waitDone();		// until we reach the end of the surrounding line next to the box
	return true;
}

bool navdeploy_loopback() {		// Run when leaving one row of boxes to loop back to the other row of boxes
	deploy_start();				// start priming sensor on deployer
	if (!nav_linefollowIntersection())	// linefollow until intersection before first box is reached
		return false;
	return true;
}

bool navdeploy_middle() {		// Run when navigating the space between the two boxes on either row of the course
	drive_rturnDeg(50, 43);	// turn right from back corner of box to intercept centerline
	drive_fd(60);				// drive forward
	drive_waitDist(12);		// wait a bit to leave line we were just on
	linefollow_waitLine(3, 4);	// stop when we intersect the middle line connecting the space between the boxes
	drive_waitDist(2);			// go a little further to center on line
	drive_cStop();			// wait for bouncing to settle
	drive_lturnDeg(50, 60);	// turn left to face second box on current side of course
	if (!linefollow_start(60))	// linefollow
		return false;
	linefollow_waitDone();		// until we reach the intersection
	drive_cStop();
	drive_bkDist(40, 14);		// backup to linefollow again for second chance to line ourselves up better with the box
	if (!linefollow_start(60))	// linefollow
		return false;
	linefollow_waitDone();		// until we reach the intersection for the second time (lined up more correctly)
	drive_cStop();
	return true;
}

void navdeploy_end() {			// Run after second box has been navigated around on one side to get to loopback position
	drive_rturnDeg(50, 30);		// Turn to intersect loopback line
	drive_fd(60);				// Start going towards the loopback line
	drive_waitDist(10);			// Wait a little before looking for the line to escape line currently on
	linefollow_waitLine(3, 4);	// Wait for the middle sensors to see a line, this is our first crossover
	drive_waitDist(10);			// Keep going to get off that line
	linefollow_waitLine(6, 7);	// Wait until the right side of the sensor is triggered (this will be loopback line to follow)
	drive_cStop();
	drive_rturnDeg(60, 45);		// Turn to face direction on new loopback line
}
