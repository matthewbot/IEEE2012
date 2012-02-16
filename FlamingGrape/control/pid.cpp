#include "control/pid.h"
#include "debug/debug.h"
#include <math.h>

void pid_initState(PIDState &state) {
	state.sum = 0;
	state.prev = 0;
}

float pid_update(PIDState &state, const PIDGains &gains, float error, float dt, PIDDebug *debug) {
	float p = gains.p*error;
	
	state.sum += error*dt;
	if (state.sum > gains.maxi)
		state.sum = gains.maxi;
	else if (state.sum < -gains.maxi)
		state.sum = -gains.maxi;
	float i = gains.i*state.sum;
	
	float d = gains.d*(error - state.prev)/dt;
	state.prev = error;

	if (debug) {
		debug->p_out = p;
		debug->i_out = i;
		debug->d_out = d;
	}

	return p+i+d;
}

void pid_printDebug(float out, float error, const PIDDebug &d) {
	debug_out("E % .3f O % .3f P % .3f I % .3f D % .3f\n", error, out, d.p_out, d.i_out, d.d_out);
}
