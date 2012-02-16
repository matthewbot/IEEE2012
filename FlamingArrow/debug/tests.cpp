#include "debug/tests.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include <util/delay.h>
#include <stdio.h>

void tests_pwm() {
	for (int16_t pwm = 0; pwm < motor_maxpwm; pwm++)
		tests_pwm_single(pwm);
		
	motor_setpwm(MOTOR_LEFT, 0);
	motor_setpwm(MOTOR_RIGHT, 0);
	_delay_ms(2000);
	
	for (int16_t pwm = 0; pwm > -motor_maxpwm; pwm--)
		tests_pwm_single(pwm);
		
	motor_setpwm(MOTOR_LEFT, 0);
	motor_setpwm(MOTOR_RIGHT, 0);
}

void tests_pwm_single(int16_t pwm) {
	enc_reset(MOTOR_LEFT);
	enc_reset(MOTOR_RIGHT);
	motor_setpwm(MOTOR_LEFT, pwm);
	motor_setpwm(MOTOR_RIGHT, pwm);
	_delay_ms(10);
	printf("%d %d %d\n", pwm, enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
}
