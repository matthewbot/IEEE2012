#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "control/motorcontrol.h"
#include "pid.h"
#include "util.h"

#include "control/linefollow.h"

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

float turn_speed = 0.;
bool on_line = false;

void linefollow_sensorUpdate(const uint16_t *readings) {
	if(!enabled)
		return;

	float lineness;
	float pos = get_line_pos(readings, &lineness);

	if(on_line && lineness < 0.00013)
		on_line = false;
	else if(!on_line && lineness > 0.00023)
		on_line = true;

	if(on_line) {
		float line_pid_change = pid_update(line_pid, line_pidcoefs, 0, pos, .01); // TODO: compute dt

		turn_speed = line_pid_change*line_pid_change*line_pid_change;
	} else {
		turn_speed = 30*sign(turn_speed);
	}

	//printf("%f %f %f\n", speed, pos, fabs(pos));
	printf("%f\n", (double)lineness);
	//motorcontrol_setvel(0, 15 + turn_speed);
	//motorcontrol_setvel(1, 15 - turn_speed);
}

void linefollow_setEnabled(bool newenabled) {
	if (enabled == newenabled)
		return;

	if (newenabled)
		pid_initstate(line_pid); //Enable PID for linesensor
	enabled = newenabled;
}
