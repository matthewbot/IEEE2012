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
		
/*		if (i == 0) {		// Twitch for temperature box
			_delay_ms(1000);
			drive_lturn_deg(30, 5);
		}*/
		
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
	drive_stop();				// stop
	_delay_ms(300);				// wait for bouncing to stop
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
	drive_stop();
	_delay_ms(300);			// wait for bouncing to settle
	drive_lturnDeg(50, 60);	// turn left to face second box on current side of course
	if (!linefollow_start(60))	// linefollow
		return false;
	linefollow_waitDone();		// until we reach the intersection
	_delay_ms(300);
	drive_bkDist(40, 14);		// backup to linefollow again for second chance to line ourselves up better with the box
	if (!linefollow_start(60))	// linefollow
		return false;
	linefollow_waitDone();		// until we reach the intersection for the second time (lined up more correctly)
	_delay_ms(300);
	return true;
}

void navdeploy_end() {			// Run after second box has been navigated around on one side to get to loopback position
	drive_rturnDeg(50, 40);	// turn right to intercept loopback line behind second box
	drive_fd(60);				// drive forward towards line
	drive_waitDist(10);		// go a little bit to make sure we don't intercept line we're already on (back corner of second box)
	linefollow_waitLine(0, 3);	// wait until line (now we're on the loopback start line)
	drive_stop();
	_delay_ms(300);
	drive_lturnDeg(50, 60);	// turn left to face the direction we want to go on the loopback line
/***************************
** ^ Make into function ^ **
***************************/
}
