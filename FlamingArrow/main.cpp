#include "init.h"
#include "debug.h"
#include "motorcontrol.h"
#include "motor.h"
#include "linesensor.h"
#include <util/delay.h>
#include <math.h>

uint16_t linesensors[8] = {0};

int linesteer = 0;
int current = 0;

int main() {
	init();
	debug_puts("Setting motor velocities\r\n");
	motor_setpwm(0,0 );
	motor_setpwm(1,0);
    line_cal();
	while (true) {
		line_follow();
		_delay_ms(100);
	}
}
