#include "init.h"
#include "debug.h"
#include "motorcontrol.h"
#include "motor.h"
#include "linesensor.h"
#include <util/delay.h>
#include <math.h>
#include "enc.h"

float get_line_pos() {
	float light_levels[8];
	for(int i=0; i<8; i++)
		light_levels[i] = 1./(1. + linesensor_get(i));
	
	float sum = 0., total = 0.;
	for(int i=0; i<8; i++) {
		sum += light_levels[i]*i;
		total += light_levels[i];
	}
	
	return sum/total/7 - .5;
}

int main() {
	init();

	

	
	while (true) {
		//motor_setpwm(0, 500);
		//motor_setpwm(1, 500);
		//debug_printf("%i %i\r\n", enc_get(0), enc_get(1));
		//continue;
		float pos = get_line_pos();
		//pos = pos*pos*pos;
		motorcontrol_setvel(0, 1 - 55*pos);
		motorcontrol_setvel(1, 1 + 55*pos);
		_delay_ms(10);
		debug_printf("%u %u %u %u %u %u %u %u %f\r\n",
			linesensor_get(0),
			linesensor_get(1),
			linesensor_get(2),
			linesensor_get(3),
			linesensor_get(4),
			linesensor_get(5),
			linesensor_get(6),
			linesensor_get(7),
			pos
		);
	}
}
