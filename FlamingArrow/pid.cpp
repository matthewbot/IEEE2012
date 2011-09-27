#include <math.h>

#include "pid.h"

void pid_new(pid_obj_t* pid_obj, float p, float i, float i_decay, float d) {
	pid_obj->p = p;
	pid_obj->i = i; pid_obj->i_decay = i_decay; pid_obj->i_sum = 0;
	pid_obj->d = d; pid_obj->d_active = 0;
}

float pid_update(pid_obj_t* p, float desired, float measured, float dt) {
	float error = desired - measured;
	
	float res = p->p * error + p->i * p->i_sum + p->d * (p->d_active ? (error - p->d_last)/dt : 0);
	
	p->i_sum += error*dt;
	p->d_last = error;
	p->d_active = true;
	
	p->i_sum = p->i_sum * exp(-dt * p->i_decay);
	
	return res;
}
