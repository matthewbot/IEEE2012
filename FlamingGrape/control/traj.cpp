#include "control/traj.h"
#include "control/motorcontrol.h"
#include "debug/debug.h"
#include "hw/tick.h"
#include "util.h"
#include <math.h>
#include <stdio.h>

struct TrajData {
	float vmax;
	float amax;
	float h; // distance
	float v0; // initial velocity
	float v1; // final velocity
	
	float Ta; // length of accel
	float Td; // length of deaccel
	float T; // total time
	float Vv; // max velocity
};

static TrajData traj[motorcontrol_count];
static volatile uint32_t tickstart;

enum Mode {
	MODE_DISABLED,
	MODE_TRAJECTORY,
	MODE_RAMP
};
static volatile Mode mode;

static void prepareTrajectoryData(TrajData &d);
static float computeTrajectory(const TrajData &d, float t, bool &done);
static float computeRamp(const TrajData &d, float t, bool &done);

void traj_goDist(float ldist, float lfinalrps, float lvmax, float lamax, float rdist, float rfinalrps, float rvmax, float ramax) {
	mode = MODE_DISABLED;
	
	traj[MOTOR_LEFT].vmax = lvmax;
	traj[MOTOR_LEFT].amax = ldist > 0 ? lamax : -lamax;
	traj[MOTOR_LEFT].h = ldist;
	traj[MOTOR_LEFT].v0 = motorcontrol_getRPSDesired(MOTOR_LEFT);
	traj[MOTOR_LEFT].v1 = lfinalrps;
	prepareTrajectoryData(traj[MOTOR_LEFT]);
	
	traj[MOTOR_RIGHT].vmax = rvmax;
	traj[MOTOR_RIGHT].amax = rdist > 0 ? ramax : -ramax;
	traj[MOTOR_RIGHT].h = rdist;
	traj[MOTOR_RIGHT].v0 = motorcontrol_getRPSDesired(MOTOR_RIGHT);
	traj[MOTOR_RIGHT].v1 = rfinalrps;
	prepareTrajectoryData(traj[MOTOR_RIGHT]);
	
	tickstart = tick_getCount();
	mode = MODE_TRAJECTORY;
}

void traj_goVel(float lfinalrps, float lamax, float rfinalrps, float ramax) {
	mode = MODE_DISABLED;
	
	traj[MOTOR_LEFT].v0 = motorcontrol_getRPSDesired(MOTOR_LEFT);
	traj[MOTOR_LEFT].v1 = lfinalrps;
	traj[MOTOR_LEFT].amax = traj[MOTOR_LEFT].v1 > traj[MOTOR_LEFT].v0 ? lamax : -lamax;
	traj[MOTOR_RIGHT].v0 = motorcontrol_getRPSDesired(MOTOR_RIGHT);
	traj[MOTOR_RIGHT].v1 = rfinalrps;
	traj[MOTOR_RIGHT].amax = traj[MOTOR_RIGHT].v1 > traj[MOTOR_RIGHT].v0 ? ramax : -ramax;
	
	tickstart = tick_getCount();
	mode = MODE_RAMP;
}

void traj_wait() {
	while (mode != MODE_DISABLED) { }
}

void traj_stop() {
	mode = MODE_DISABLED;
	for (int i=0; i<motorcontrol_count; i++)
		motorcontrol_setRPS(i, 0);
}

void traj_tick() {
	if (mode == MODE_DISABLED)
		return;
	
	float t = (float)(tick_getCount() - tickstart) / (float)TICK_HZ;
	
	bool done=true;
	for (int i=0; i<motorcontrol_count; i++) {
		TrajData &d = traj[i];
		
		float rps;
		bool partdone;
		if (mode == MODE_TRAJECTORY)
			rps = computeTrajectory(d, t, partdone);
		else if (mode == MODE_RAMP)
			rps = computeRamp(d, t, partdone);
		else
			continue;
			
		if (!partdone)
			done = false;
		
		motorcontrol_setRPS(i, rps);
	}
	
	motorcontrol_setEnabled(true);
	
	if (done)
		mode = MODE_DISABLED;
}

static float computeTrajectory(const TrajData &d, float t, bool &done) {
	done = false;	
	if (t < d.Ta) {
		return d.v0 + ((d.Vv - d.v0)/d.Ta)*t;
	} else if (t < d.T - d.Td) {
		return d.Vv;
	} else if (t < d.T) {
		return d.v1 + ((d.Vv - d.v1)/d.Td)*(d.T-t);
	} else {
		done = true;
		return 0;
	}
}

static float computeRamp(const TrajData &d, float t, bool &done) {
	done = false;
	
	float v = d.v0 + d.amax*t;
	if (d.amax > 0) {
		if (v >= d.v1) {
			done = true;
			return d.v1;
		}
	} else {
		if (v <= d.v1) {
			done = true;
			return d.v1;
		}
	}
	
	return v;
}

static void prepareTrajectoryData(TrajData &d) {
	float vlim = sqrtf(d.h*d.amax + (d.v0*d.v0 + d.v1*d.v1)/2);
	if (d.vmax < 0)
		vlim = -vlim;
		
	if (fabsf(vlim) >= fabsf(d.vmax)) {
		d.Ta = (d.vmax - d.v0) / d.amax;
		d.Td = (d.vmax - d.v1) / d.amax;
		d.T  = d.h/d.vmax + d.vmax/(2*d.amax)*sqrf(1-d.v0/d.vmax)
		     + d.vmax/(2*d.amax)*sqrf(1-d.v1/d.vmax);
		d.Vv = d.vmax;
	} else {
		d.Ta = (vlim - d.v0) / d.amax;
		d.Td = (vlim - d.v1) / d.amax;
		d.T  = d.Ta + d.Td;
		d.Vv = vlim;
	}
}
