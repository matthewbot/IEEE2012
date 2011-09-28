#include "init.h"
#include "debug.h"
#include "motorcontrol.h"
#include "motor.h"
#include <util/delay.h>
#include <math.h>

int main() {
	init();
	debug_puts("Setting motor velocities\r\n");
	motorcontrol_setvel(0, 2.3);
	motorcontrol_setvel(1, 2.3);

	while (true) {
		debug_printf("Motor pwm %4d %4d vel %.2f %.2f\r\n", motor_getpwm(0), motor_getpwm(1), motorcontrol_getvel(0), motorcontrol_getvel(1));
		_delay_ms(1000);
	}
}
