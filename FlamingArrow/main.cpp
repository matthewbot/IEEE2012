#include "init.h"
#include "debug.h"
#include "motorcontrol.h"
#include "motor.h"
#include "linesensor.h"
#include <util/delay.h>
#include <math.h>
#include "enc.h"



int main() {
	init();

	

	
	while (true) {
		//motor_setpwm(0, 500);
		//motor_setpwm(1, 500);
		//debug_printf("%i %i\r\n", enc_get(0), enc_get(1));
		//continue;
		
		//pos = pos*pos*pos;
		line_follow();
		_delay_ms(10);
		debug_printf("%u %u %u %u %u %u %u %u\r\n",
			linesensor_get(0),
			linesensor_get(1),
			linesensor_get(2),
			linesensor_get(3),
			linesensor_get(4),
			linesensor_get(5),
			linesensor_get(6),
			linesensor_get(7)
		);
	}
}
