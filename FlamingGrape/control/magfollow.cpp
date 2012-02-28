#include "control/magfollow.h"
#include "hw/mag.h"
#include "hw/tick.h"
#include "control/pid.h"
#include "control/drive.h"
#include "control/motorcontrol.h"
#include "debug/debug.h"
#include "util.h"
#include <math.h>


static volatile bool enabled = false;
static volatile float heading;
static volatile float vel;
static volatile float error_filter;
static volatile bool debug;
static PIDState pidstate;
static PIDGains pidgains = {200, 0, 12};
static float heading_offset;			// heading offset value
static MagCal magcal = {-97.5, -81, 0.89606};		// x_offset, y_offset, y_scale

void magfollow_start(float new_vel, float new_heading) {
	pid_initState(pidstate);
	heading = new_heading;
	vel = new_vel;
	enabled = true;
}

void magfollow_stop() {
	enabled = false;
	drive_stop();
}

float magfollow_getHeading() {
	MagReading reading = mag_getReading();
	float x = reading.x - magcal.x_offset;
	float y = (reading.y - magcal.y_offset)*magcal.y_scale;
	return anglewrap(atan2(y, x) + heading_offset);
}

void magfollow_setHeading(float desired_heading) {
	heading_offset += desired_heading - magfollow_getHeading();
	heading_offset = anglewrap(heading_offset);
}

void magfollow_tick() {
	if (!enabled) {
		return;
	}
	
	float error = magfollow_getHeading() - heading;
	error = anglewrap(error);
	
	error_filter = .85f*error_filter + .15f*error;

	PIDDebug piddebug;
	float out = pid_update(pidstate, pidgains, error_filter, TICK_DT, &piddebug);

	if (debug)
		pid_printDebug(out, error_filter, piddebug);
	
	drive_steer(out, vel);

}

void magfollow_setDebug(bool new_debug) {
	debug = new_debug;
}

void magfollow_setCal(const MagCal &new_magcal) {
	magcal = new_magcal;
}

const MagCal &magfollow_getCal() {
	return magcal;
}

PIDGains magfollow_getGains() {
	return pidgains;
}

void magfollow_setGains(const PIDGains &newpidgains) {
	pidgains = newpidgains;
}

