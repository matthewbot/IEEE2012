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
}

void navdeploy_deploy() {
	deploy_waitDone();
	drive_fd_dist(50, 20);
	drive_fd(20);
	deploy_out(true);
	_delay_ms(2000);
	drive_bk(4);
	_delay_ms(500);
	deploy_off();
	drive_fd(4);
	_delay_ms(500);
	drive_bk(4);
	_delay_ms(500);
	deploy_start();
}

bool navdeploy_aroundBox() {
	drive_bk_dist(30, 15);
	drive_lturn_deg(60, 15);
	drive_fd_dist(60, 26);
	drive_rturn_deg(60, 60);
	
	if (!linefollow_start(60, .4))
		return false;
	linefollow_waitDone();
}

bool navdeploy_loopback() {
	deploy_start();		
	if (!nav_linefollowIntersection())
		return false;
	drive_stop();
	return true;
}

bool navdeploy_middle() {
	drive_rturn_deg(50, 35);
	drive_fd_dist(60, 20);
	drive_fd(60);
	linefollow_waitLine();
	drive_lturn_deg(50, 80);
	if (!linefollow_start(60))
		return false;
	linefollow_waitDone();
	drive_stop();
	return true;
}
