#ifndef PID_H_
#define PID_H_

#include <stdlib.h>

struct PIDGains {
	float p, i, d;
	float maxi;
};

struct PIDState {
	float sum;
	float prev;
};

struct PIDDebug {
	float p_out;
	float i_out;
	float d_out;
};

void pid_initState(PIDState &state);
float pid_update(PIDState &state, const PIDGains &gains, float error, float dt, PIDDebug *debug=NULL);

void pid_printDebug(float out, float error, const PIDDebug &debug);

#endif /* PID_H_ */
