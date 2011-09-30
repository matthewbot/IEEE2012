#include "linesensor.h"
#include "motorcontrol.h"
#include "pid.h"

float pos, line_pid_change;
float line_desired = 0;
float line_dt = .01;
PIDState line_pid;
static const PIDCoefs line_pidcoefs = {55, 0, 0.01, 0};


void linefollow_init() {
	 //Enable PID for linesensor
	 pid_initstate(line_pid);
}

float get_line_pos() {
	float light_levels[8];
	for(int i=0; i<8; i++)
		light_levels[i] = 1./(1. + linesensor_get(i));
	
	float min_level = light_levels[0];
	for(int i=0; i<8; i++)
		if(light_levels[i] < min_level)
			min_level = light_levels[i];
	
	for(int i=0; i<8; i++)
		light_levels[i] -= min_level;
	
	float sum = 0., total = 0.;
	for(int i=0; i<8; i++) {
		sum += light_levels[i]*light_levels[i]*i;
		total += light_levels[i]*light_levels[i];
	}
	
	if(total == 0)
		return 0;
	return sum/total/7 - .5; // range is [-0.5, +0.5]
}

//Desired value should be zero after eqn analysis

void linefollow_follow() {
	pos = get_line_pos();
	line_pid_change = pid_update(line_pid, line_pidcoefs, line_desired, pos, line_dt);
	motorcontrol_setvel(0, 1 + line_pid_change);
	motorcontrol_setvel(1, 1 - line_pid_change);
}
