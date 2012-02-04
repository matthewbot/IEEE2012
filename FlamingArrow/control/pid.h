#ifndef PID_H_
#define PID_H_

#include <stdlib.h>

struct PIDCoefs {
	float p, i, d;
	float maxi;
};

struct PIDState {
	float error_sum;
	float error_last;
	bool d_active;
};

void pid_initstate(PIDState &state);
float pid_update(PIDState &state, const PIDCoefs &coefs, float desired, float measured, float dt, float *d=NULL);

#endif /* PID_H_ */
