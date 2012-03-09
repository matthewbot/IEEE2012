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

void navdeploy_deploy() {
	deploy_waitDone();
	_delay_ms(500);
	drive_fd_dist(20, 15);
	deploy_out(true);
	_delay_ms(2000);
	
	for (int i = 0; i < 2; i++) {
		drive_bk_dist(20, 5);
		deploy_off();
		drive_fd_dist(20, 7);
	}
	
	drive_bk(4);
	linefollow_waitLine();
	
	deploy_start();
}

bool navdeploy_aroundBox() {
	drive_bk_dist(30, 10);
	drive_lturn_deg(60, 55);
	drive_fd(60);
	_delay_ms(300);
	linefollow_waitLine(7, 7);
	_delay_ms(80);
	linefollow_waitLine(7, 7);
	drive_stop();
	drive_rturn_deg(60, 45);
	
	if (!linefollow_start(60, .4))
		return false;
	linefollow_waitDone();
	return true;
}

bool navdeploy_loopback() {
	deploy_start();		
	if (!nav_linefollowIntersection())
		return false;
	return true;
}

bool navdeploy_middle() {
	drive_rturn_deg(50, 40);
	drive_fd(60);
	drive_wait_dist(12);
	linefollow_waitLine(3, 4);
	drive_wait_dist(2);
	drive_stop();
	drive_lturn_deg(50, 60);
	if (!linefollow_start(60))
		return false;
	linefollow_waitDone();
	drive_stop();
	drive_bk_dist(40, 14);
	if (!linefollow_start(60))
		return false;
	linefollow_waitDone();
	drive_stop();
	return true;
}

void navdeploy_end() {
	drive_rturn_deg(50, 40);
	drive_fd(60);
	drive_wait_dist(10);
	linefollow_waitLine(0, 3);
	drive_stop();
	drive_lturn_deg(50, 70);
}
