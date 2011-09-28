#ifndef PID_H_
#define PID_H_

struct PIDCoefs {
	float p, i, d;
	float i_decay;
};

struct PIDState {
	float error_sum;
	float error_last;
	bool d_active;
};

void pid_initstate(PIDState &state);
float pid_update(PIDState &state, const PIDCoefs &coefs, float desired, float measured, float dt);

#endif /* PID_H_ */
