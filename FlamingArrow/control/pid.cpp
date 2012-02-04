#include <math.h>

#include "pid.h"

void pid_initstate(PIDState &state) {
	state.error_sum = 0; // integrator starts at zero
	state.d_active = false; // the d term starts off inactive since we need two error samples to compute a derivative
}

float pid_update(PIDState &s, const PIDCoefs &c, float desired, float measured, float dt, float *computed_d) {
	float error = desired - measured; // compute error
	// compute output. D term is only included if its active
	float d = (s.d_active == 0 ? (error - s.error_last)/dt : 0);
	if(computed_d)
		*computed_d = d;
	float out = c.p * error + c.i * s.error_sum + c.d * d;
	
	s.error_sum += error*dt; // integrate the error
	if (s.error_sum > c.maxi)
		s.error_sum = c.maxi;
	else if (s.error_sum < -c.maxi)
		s.error_sum = -c.maxi;
		
	s.error_last = error; // save this sample to compute the next derivative
	s.d_active = true; // D term is now active

	return out;
}

