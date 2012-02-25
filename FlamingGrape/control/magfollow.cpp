#include "control/magfollow.h"
#include "hw/mag.h"
#include "hw/tick.h"
#include "control/pid.h"
#include "control/drive.h"
#include "control/motorcontrol.h"
#include "debug/debug.h"
#include <math.h>


static volatile bool enabled = false;
static volatile float heading;
static volatile float vel;
static volatile bool debug;
static PIDState pidstate;
static PIDGains pidgains = {10, 0, 0};
static float heading_offset;			// heading offset value
static MagCal magcal = {1, 1, 1};		// x_offset, y_offset, y_scale

void magfollow_start(float new_vel, float new_heading) {
	pid_initState(pidstate);
	heading = new_heading;
	vel = new_vel;
	enabled = true;
}

void magfollow_setDebug(bool new_debug) {
	debug = new_debug;
}

void magfollow_stop() {
	enabled = false;
	motorcontrol_setEnabled(false);
}

float magfollow_getHeading() {
	MagReading reading = mag_getReading();
	float x = reading.x - magcal.x_offset;
	float y = (reading.y - magcal.y_offset)*magcal.y_scale;
	return atan2(y, x) - heading_offset;
}

void magfollow_setHeading(float desired_heading) {
	float error = desired_heading - magfollow_getHeading();
	heading_offset = error;
}

void magfollow_tick() {
	if (!enabled) {
		return;
	}
	
	float error = heading - magfollow_getHeading();
	if (error > M_PI) {
		error -= 2*M_PI;
	} else if (error < -M_PI) {
		error += 2*M_PI;
	}
	
	if (error > M_PI/2) {
		drive_rturn(20);
	} else if (error < -M_PI/2) {
		drive_lturn(20);
	} else {
		PIDDebug piddebug;
		float out = pid_update(pidstate, pidgains, error, TICK_DT, &piddebug);
	
		if (debug)
			pid_printDebug(out, error, piddebug);
		
		drive_steer(out, vel);
	}
}

