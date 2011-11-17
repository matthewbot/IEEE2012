#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "motorcontrol.h"
#include "pid.h"
#include "util.h"

#include "linefollow.h"

bool enabled = false;

PIDState line_pid;
static const PIDCoefs line_pidcoefs = {4, 0, 0.1, 0};

float get_line_pos(const uint16_t *readings, float *lineness) {
	float light_levels[8];
	for(int i=0; i<8; i++)
		light_levels[i] = 1./(1. + readings[i]);
	
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
	
	if(lineness) {
		*lineness = 0.;
		for(int i=0; i<8; i++) {
			if(light_levels[i] > *lineness)
				*lineness = light_levels[i];
		}
	}
	
	if(total == 0)
		return 0;
	return 2*(sum/total/7 - .5); // range is [-1, +1]
}

//Desired value should be zero after eqn analysis

void linefollow_sensorUpdate(const uint16_t *readings) {
	if(!enabled)
		return;
	
	float lineness;
	float pos = get_line_pos(readings, &lineness);
	if(lineness < 0.0001) {
		motorcontrol_setvel(0, 30);
		motorcontrol_setvel(1, -30);
		pid_initstate(line_pid);
		return;
	}

	float line_pid_change = pid_update(line_pid, line_pidcoefs, 0, pos, .01); // TODO: compute dt

	float turn_speed = line_pid_change*line_pid_change*line_pid_change;
	//printf("%f %f %f\n", speed, pos, fabs(pos));
	printf("%f\n", (double)lineness);
	motorcontrol_setvel(0, 30 + turn_speed);
	motorcontrol_setvel(1, 30 - turn_speed);
}

void linefollow_setEnabled(bool enbld) {
	if(enbld && !enabled) // false -> true
		pid_initstate(line_pid); //Enable PID for linesensor
	
	if(!enbld && enabled) { // true -> false
		util_cli_lo();
		motorcontrol_disable(0);
		motorcontrol_disable(1);
		util_sei_lo();
	}
	
	enabled = enbld;
}
