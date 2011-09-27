#include "init.h"
#include "debug.h"
#include "linesensor.h"
#include <util/delay.h>
#include "motor.h"
#include "enc.h"
#include "pid.h"
#include <math.h>

int main() {
	init();
	pid_obj_t p;
	pid_new(&p, .8, .1, .4, .01);
	float t = 0;

	while (true) {
		debug_printf("\r\n");
		int16_t e0 = enc_get(0);
		float rps = e0/563.03/.05;
		debug_printf("enc: %i rps: %f\r\n", e0, (double)rps);
		debug_printf("int: %f\r\n", (double)p.i_sum);

		float out = pid_update(&p, 1.3*sin(t), rps, .05);
		debug_printf("out: %f\r\n", (double)out);
		if(out < -1) out = -1;
		if(out > 1) out = 1;
		int16_t pwm = 1024*out;
		debug_printf("pwm: %i\r\n", pwm);
		motor_setpwm(1, pwm);

		enc_reset();
		_delay_ms(50);
		t += .05;
	}
}
