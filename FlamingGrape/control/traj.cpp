#include "control/traj.h"
#include "control/motorcontrol.h"
#include "debug/debug.h"
#include "hw/tick.h"
#include "util.h"
#include <math.h>
#include <stdio.h>

struct TrajData {
	float amax;
	float vmax;
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
static volatile bool enabled;

static void computeTrajStart(TrajData &d);
static float computeTrajRPS(TrajData &d, float t);

void traj_setup(Motor mot, float dist, float final_rps, float vmax, float amax) {
	TrajData &d = traj[mot];
	d.amax = amax;
	d.vmax = vmax;
	d.h = dist;
	d.v0 = motorcontrol_getrpsDesired(mot);
	d.v1 = final_rps;
	
	computeTrajStart(d);
}

void traj_setEnabled(bool newenabled) {
	enabled = newenabled;
	if (enabled) {
		tickstart = tick_getCount();
	} else {
		for (int i=0; i<motorcontrol_count; i++)
			motorcontrol_setrps(i, 0);
	}
}

void traj_wait() {
	while (enabled) { }
}

void traj_tick() {
	if (!enabled)
		return;
	
	float t = (float)(tick_getCount() - tickstart) / (float)TICK_HZ;
	
	bool done=true;
	for (int i=0; i<motorcontrol_count; i++) {
		TrajData &d = traj[i];
		
		float rps = computeTrajRPS(d, t);
		motorcontrol_setrps(i, rps);
		
		if (t < d.T)
			done = false;
	}
	
	motorcontrol_setEnabled(true);
	
	if (done)
		enabled = false;
}

static float computeTrajRPS(TrajData &d, float t) {	
	if (t < d.Ta) {
		return d.v0 + ((d.Vv - d.v0)/d.Ta)*t;
	} else if (t < d.T - d.Td) {
		return d.Vv;
	} else if (t < d.T) {
		return d.v1 + ((d.Vv - d.v1)/d.Td)*(d.T-t);
	} else {
		return 0;
	}
}

static void computeTrajStart(TrajData &d) {
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
