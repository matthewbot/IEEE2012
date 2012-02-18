#include "competition/nav.h"
#include "control/drive.h"
#include "control/linefollow.h"
#include "hw/adc.h"
#include <stdio.h>

#include <util/delay.h>

bool nav_loopback() {
	uint8_t turncount = 0;
	
	while (true) {
		linefollow_start(60);
		
		while (!linefollow_isDone()) {
			_delay_ms(10);
			float reading = adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE);
			printf("%f\n", reading);
			if (turncount >= 2 && reading < 20) {
				printf("Rangefinder\n");
				drive_stop();
				return false;
			}
		}
		
		if (linefollow_getLastFeature() == FEATURE_NOLINE) {
			if (linefollow_getLastTurn() == TURN_LEFT) {
				drive_lturn_deg(80, 50);
				turncount++;
			} else if (linefollow_getLastTurn() == TURN_RIGHT) {
				drive_rturn_deg(80, 50);
				turncount++;
			} else {
				break;
			}
		} else {
			break;
		}
	}
	
	printf("Error\n");
	drive_stop();
	return false;
}
