#include <math.h>
#include "pid.h"

void pid_initstate(PIDState &state) {
	state.error_sum = 0;
	state.d_active = false;
}

float pid_update(PIDState &s, const PIDCoefs &c, float desired, float measured, float dt) {
	float error = desired - measured;
	float out = c.p * error + c.i * s.error_sum + c.d * (s.d_active ? (error - s.error_last)/dt : 0);

	s.error_sum += error*dt;
	s.error_sum *= exp(-dt * c.i_decay);
	s.error_last = error;
	s.d_active = true;

	return out;
}

