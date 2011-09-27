#ifndef PID_H_
#define PID_H_

typedef struct {
	float p;
	float i, i_decay, i_sum;
	float d, d_last;
	bool d_active;
} pid_obj_t;

void pid_new(pid_obj_t* pid_obj, float p, float i, float i_decay, float d);

float pid_update(pid_obj_t* p, float desired, float measured, float dt);

#endif /* PID_H_ */
