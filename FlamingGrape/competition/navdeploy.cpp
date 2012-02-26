#include "competition/navdeploy.h"
#include "competition/nav.h"
#include "control/drive.h"
#include "control/deploy.h"
#include "control/linefollow.h"
#include "debug/debug.h"
#include <util/delay.h>

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

void navdeploy_aroundBox() {
	drive_bk_dist(30, 15);
	drive_lturn_deg(60, 15);
	drive_fd_dist(60, 26);
	drive_rturn_deg(60, 60);
	
	linefollow_start(60, .4);
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
	linefollow_start(60);
	linefollow_waitDone();
	drive_stop();
	return true;
}
