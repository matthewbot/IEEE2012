#include "control/linefollow.h"
#include "control/motorcontrol.h"
#include "control/pid.h"
#include "control/drive.h"
#include "hw/sensors.h"
#include "util.h"
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

static const PIDCoefs pidcoefs = {4, 0, 0.1, 0};
static uint16_t readings[linesensor_count];
static LineFollowResults results;

void linefollow_computeResults(const uint16_t *readings, LineFollowResults &results) {
	for (int i=0; i<8; i++)
		results.raw_light[i] = 1./(1. + readings[i]);

	results.raw_min = results.raw_light[0];
	for (int i=0; i<8; i++)
		if (results.raw_light[i] < results.raw_min)
			results.raw_min = results.raw_light[i];

	for (int i=0; i<8; i++)
		results.light[i] = results.raw_light[i] /*- results.raw_min*/;

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
	
	int lostctr=0;
	float prevout=0;

	while (!sensors_readBump()) {
		linesensor_read(readings);
		linefollow_computeResults(readings, results);
		
		if (results.squaretotal > 1e-06) {
			printf("!!!!!!Intersection!\n");
			return;
		}

		if (results.max < 0.0001) {
			lostctr++;
			if (lostctr > 3) {
				printf("!!!!!!Lost line\n");
				drive_stop();
				_delay_ms(200);
				drive_bk(10);
				_delay_ms(100);
				linefollow_wait_line();
				
				drive_rturn_deg(50, 10);
				
				
				if (results.max < 0.0001)
					drive_lturn_deg(140, 10);
				
			}
		} else {
			lostctr = 0;
		}
		
		//float out = -pid_update(pid, pidcoefs, offset, results.steer, .01); 
		float out = results.steer - offset;
		prevout = out;
		
		//printf("out: %f\nmax: %f\n", (double)out, (double)results.max);
		if (out > 1)
			out = 1;
		else if (out < -1)
			out = -1;
		
		
		drive_steer(out, 40);
		
		_delay_ms(25);
	}
}

void linefollow_wait_line() {
	while (true) {
		linesensor_read(readings);
		linefollow_computeResults(readings, results);
		
		if (results.max > 0.00005)
			break;
			
		_delay_ms(20);
	}
}
