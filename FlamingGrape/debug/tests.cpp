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

void tests_PWM() {
	for (int16_t PWM = 0; PWM < motor_maxPWM; PWM++)
		tests_PWM_single(PWM);
		
	motor_setPWM(MOTOR_LEFT, 0);
	motor_setPWM(MOTOR_RIGHT, 0);
	_delay_ms(2000);
	
	for (int16_t PWM = 0; PWM > -motor_maxPWM; PWM--)
		tests_PWM_single(PWM);
		
	motor_setPWM(MOTOR_LEFT, 0);
	motor_setPWM(MOTOR_RIGHT, 0);
}

void tests_PWM_single(int16_t PWM) {
	enc_reset(MOTOR_LEFT);
	enc_reset(MOTOR_RIGHT);
	motor_setPWM(MOTOR_LEFT, PWM);
	motor_setPWM(MOTOR_RIGHT, PWM);
	_delay_ms(10);
	printf_P(PSTR("%d %d %d\n"), PWM, enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
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
	
	float offset=0;
	controlpanel_prompt("Offset", "%f", &offset);
		
	printf_P(PSTR("Push any key to stop. "));
	linefollow_start(vel, offset);
	getchar();
	linefollow_stop();
	putchar('\n');
	
	printf_P(PSTR("Turn:\t")); linefollow_printTurn(linefollow_getLastTurn()); putchar('\n');
	printf_P(PSTR("Feat:\t")); linefollow_printFeature(linefollow_getLastFeature()); putchar('\n');
}

void tests_movingLineRead() {		// Allows line sensor to be read while motors spin (to debug issues from encoder magnets)
	drive_fd(60);
	while (true) {
		debug_resetTimer();
		LineFollowResults results = linefollow_readSensor();
		uint16_t time = debug_getTimer();

		printf_P(PSTR("Light:\t"));
		for (int i=0; i<linesensor_count; i++)
			printf_P(PSTR("%2.2f\t"), results.light[i]);
		putchar('\n');						

		printf_P(PSTR("Thresh:\t"));
		for (int i=0; i<linesensor_count; i++)
			printf_P(PSTR("%d\t"), results.thresh[i]);
		putchar('\n');

		printf_P(PSTR("Center:\t%f\n"), results.center);

		printf_P(PSTR("Turn:\t")); linefollow_printTurn(results.turn); putchar('\n');
		printf_P(PSTR("Feat:\t")); linefollow_printFeature(results.feature); putchar('\n');
		printf_P(PSTR("Time:\t%ud usec\n"), time);

		printf_P(PSTR("Push any key to take readings, or space to stop. \n"));
		char c = getchar();
		if (c == ' ') {
			drive_stop();
			return;
		}
	}
	drive_stop();
}

void tests_linesensorMin(uint16_t *min) {	
	linesensor_read(min);
	for (int i = 0; i < 200; i++) {
		uint16_t buf[8];
		linesensor_read(buf);
		for (int j = 0; j < 8; j++) {
			if (buf[j] < min[j]) {
				min[j] = buf[j];
			}
		}
		_delay_ms(20);
	}
}
