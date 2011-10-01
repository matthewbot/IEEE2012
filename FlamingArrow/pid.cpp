#include <math.h>

#include "pid.h"

void pid_initstate(PIDState &state) {
	state.error_sum = 0; // integrator starts at zero
	state.d_active = false; // the d term starts off inactive since we need two error samples to compute a derivative
}

float pid_update(PIDState &s, const PIDCoefs &c, float desired, float measured, float dt) {
	float error = desired - measured; // compute error
	// compute output. D term is only included if its active
	float out = c.p * error + c.i * s.error_sum + c.d * (s.d_active ? (error - s.error_last)/dt : 0);

	s.error_sum += error*dt; // integrate the error
	s.error_sum *= exp(-dt * c.i_decay); // we give the integrator a decay factor to help with wind-up at the expense of steady-state error
	s.error_last = error; // save this sample to compute the next derivative
	s.d_active = true; // D term is now active

	return out;
}

