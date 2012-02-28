#include "debug/tests.h"
#include "debug/debug.h"
#include "debug/controlpanel.h"
#include "control/linefollow.h"
#include "control/magfollow.h"
#include "control/drive.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include "hw/mag.h"
#include "util.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
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
	printf_P(PSTR("%d %d %d\n"), pwm, enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
}

void tests_mag() {
	MagReading first_reading = mag_getReading();
	MagReading reading;
	bool returning = false;
	while (true) {
		printf_P(PSTR("%5d %5d %5d\n"), reading.x, reading.y, reading.z);
		drive_lturn(10);
		reading = mag_getReading();

		float diff = sqrtf(squaref(reading.x - first_reading.x) + squaref(reading.y - first_reading.y));

		if (!returning && (diff > 100)) {
			returning = true;
		} else if (returning && (diff < 20)) {
			break;
		}
	}
	
	drive_stop();
}

void tests_magfollow() {
	float vel;
	if (!controlpanel_prompt("Velocity", "%f", &vel)) {
		printf_P(PSTR("Cancelled.\n"));
		return;
	}
	float heading;
	if (!controlpanel_prompt("Heading", "%f", &heading)) {
		printf_P(PSTR("Cancelled.\n"));
		return;
	}
		
	printf_P(PSTR("Push any key to stop.\n"));
	magfollow_start(vel, degtorad(heading));
	getchar();
	magfollow_stop();
}

void tests_led() {
	debug_setLED(ERROR_LED, true);
	_delay_ms(1000);
	debug_setLED(ERROR_LED, false);
	debug_setLED(YELLOW_LED, true);
	_delay_ms(1000);
	debug_setLED(YELLOW_LED, false);
	debug_setLED(GREEN_LED, true);
	_delay_ms(1000);
	debug_setLED(GREEN_LED, false);
	debug_setLED(OTHERYELLOW_LED, true);
	_delay_ms(1000);
	debug_setLED(OTHERYELLOW_LED, false);
}

void tests_linefollow() {
	float vel;
	if (!controlpanel_prompt("Velocity", "%f", &vel)) {
		printf_P(PSTR("Canceled.\n"));
		return;
	}
		
	printf_P(PSTR("Push any key to stop. "));
	linefollow_start(vel);
	getchar();
	linefollow_stop();
	putchar('\n');
	
	printf_P(PSTR("Turn:\t")); linefollow_printTurn(linefollow_getLastTurn()); putchar('\n');
	printf_P(PSTR("Feat:\t")); linefollow_printFeature(linefollow_getLastFeature()); putchar('\n');
}
