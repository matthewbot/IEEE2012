#include "control/linefollow.h"
#include "control/motorcontrol.h"
#include "control/pid.h"
#include "control/drive.h"
#include "util.h"
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

static const PIDCoefs pidcoefs = {4, 0, 0.1, 0};

void linefollow_computeResults(const uint16_t *readings, LineFollowResults &results) {
	for (int i=0; i<8; i++)
		results.raw_light[i] = 1./(1. + readings[i]);

	results.raw_min = results.raw_light[0];
	for (int i=0; i<8; i++)
		if (results.raw_light[i] < results.raw_min)
			results.raw_min = results.raw_light[i];

	for (int i=0; i<8; i++)
		results.light[i] = results.raw_light[i] - results.raw_min;

	results.squaresum = results.squaretotal = 0;
	for (int i=0; i<8; i++) {
		float l = results.light[i];
		results.squaresum += l*l*i;
		results.squaretotal += l*l;
	}

	results.max = 0;
	for (int i=0; i<8; i++) {
		if (results.light[i] > results.max)
			results.max = results.light[i];
	}

	results.steer = 2*(results.squaresum/results.squaretotal/7 - .5); 
}

void linefollow_bump(float offset) {
	PIDState pid;
	pid_initstate(pid);
	
	while (true) {
		static uint16_t readings[linesensor_count];
		linesensor_read(readings);
		
		static LineFollowResults results;
		linefollow_computeResults(readings, results);
		
		float out = pid_update(pid, pidcoefs, offset, results.steer, .01); 
		if (out > 1)
			out = 1;
		else if (out < -1)
			out = -1;
		drive_steer(out, 1.5);
		
		_delay_ms(10);
	}
}
