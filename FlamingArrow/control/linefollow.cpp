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

void linefollow_computeResults(const uint16_t *readings, LineFollowResults &results) {
	for (int i=0; i<8; i++)
		results.light[i] = 1./(1. + readings[i]);

	results.light_max = 0;
	for (int i=0; i<8; i++) {
		if (results.light[i] > results.light_max)
			results.light_max = results.light[i];
	}

	results.squaresum = results.squaretotal = 0;
	for (int i=0; i<8; i++) {
		float l = results.light[i];
		results.squaresum += l*l*i;
		results.squaretotal += l*l;
	}

	results.steer = 2*(results.squaresum/results.squaretotal/7 - .5); 
	
	if (results.squaretotal > 1e-06) {
		results.feature = FEATURE_INTERSECTION;
	} else if (results.squaretotal > 5e-05) {
		results.feature = results.steer > 0 ? FEATURE_RIGHTTURN : FEATURE_LEFTTURN;			
	} else if (results.light_max < 0.0001) {
		results.feature = FEATURE_NOLINE;
	} else {
		results.feature = FEATURE_NONE;
	}
}

void linefollow_drive(LineFollowResults &results, float offset) {
	float steer = results.steer - offset;
	
	if (steer > 1)
		steer = 1;
	else if (steer < -1)
		steer = -1;
	
	drive_steer(steer, 40);
}

static uint16_t readings[linesensor_count];
static LineFollowResults results;

void linefollow_intersection(float offset) {
	bool turnright=false;

	while (!sensors_readBump()) {
		linesensor_read(readings);
		linefollow_computeResults(readings, results);
		if (results.feature == FEATURE_INTERSECTION) {
			printf("!!!!!!Intersection!\n");
			return;
		}
		
		if (results.feature == FEATURE_RIGHTTURN) {
			printf("!!!!!!Right turn!\n");
			turnright = true;
		} else if (results.feature == FEATURE_LEFTTURN) {
			printf("!!!!!!Left turn!\n");
			turnright = false;
		}

		if (results.feature == FEATURE_NOLINE) {
			printf("!!!!!!Lost line\n");
			
			if (turnright)
				drive_rturn(10);
			else
				drive_lturn(10);
				
			linefollow_wait_line();
			continue;
		}
		
		linefollow_drive(results, offset);
		
		_delay_ms(25);
	}
}

void linefollow_wait_line() {
	while (true) {
		linesensor_read(readings);
		linefollow_computeResults(readings, results);
		
		if (results.feature != FEATURE_NOLINE)
			break;
			
		_delay_ms(20);
	}
}
