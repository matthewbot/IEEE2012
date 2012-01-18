#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#include "control/linefollow.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include "hw/temp.h"
#include "hw/adc.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "hw/sensor.h"

#include "debug/controlpanel.h"

static void sintest();
static void speedtest();
static void caltest();
static void rapidtest();

void controlpanel_init() {
	printf("Starting up\n");
}

void controlpanel() {
	while(true) {
		printf("Main > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'm':
				controlpanel_motor();
				break;
			case 's':
				controlpanel_sensor();
				break;
			case 'q':
				printf("Quitting...\n");
				return;
			default:
				printf("Unknown. Submenus: Motor, Temp, Line sensor. Commands: Quit.\n");
				break;
		}
	}
}

void controlpanel_motor() {
	while (true) {
		printf("Motor > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'w':
				drive_fd(20);
				break;
			case 'a':
				drive_lturn(10);
				break;
			case 's':
				drive_bk(20);
				break;
			case 'd':
				drive_rturn(10);
				break;
				
			case 'W':
				drive_fd_dist(16, 20);
				break;
			case 'A':
				drive_lturn_deg(90, 10);
				break;
			case 'S':
				drive_bk_dist(16, 20);
				break;
			case 'D':
				drive_rturn_deg(90, 10);
				break;
				
			case ' ':
				drive_stop();
				break;
			case 'f':
				sintest();
				break;
			case 'e':
				printf("L %i R %i\n", enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
				break;
			case 'b':
				return;
			case 't':
				speedtest();
				break;
			case 'c':
				caltest();
				break;
			case 'r':
				rapidtest();
				break;

			default:
				printf("Unknown. Commands: WASD, Forward, Encoders, Back.\n");
				break;
		}
	}
	
	motorcontrol_setEnabled(false);
}

void controlpanel_sensor() {
	uint16_t linebuf[linesensor_count];
	
	while (true) {
		printf("Sensor > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'c':
				printf("CapADC: %f\n", (double)sensor_readCapADC());
				break;

			case 'v':
				printf("VoltageADC: %f\n", (double)sensor_readVoltageADC());
				break;

			case 's':
				printf("SignalADC: %f\n", (double)sensor_readSignalADC());
				break;

			case 'a':
				for (int i=0; i<8; i++)
					printf("%d ", adc_sample(i));
				printf("\n");
				break;

			case 'l':
				linesensor_read(linebuf);
				for (int i=0; i<linesensor_count; i++)
					printf("%u ", linebuf[i]);
				printf("\n");
				break;

			case 'b':
				return;
		}
	}
}

static void sintest() {
	motorcontrol_setrps(0, 0);
	motorcontrol_setrps(1, 0);
	motorcontrol_setDebug(true);
	_delay_ms(1000);
	{ 
		float t = 0;
		while(true) {
			t += .01;
			motorcontrol_setrps(0, 2+sin(t*3));
			_delay_ms(10);
		}
	}
	getchar();
	motorcontrol_setDebug(false);
	motorcontrol_setEnabled(false);
}

static void speedtest() {
	printf("Motor Test > ");
	for (int i = -1024; i <= 1024; i++) {
		motor_setpwm(0, i);
		motor_setpwm(1, i);
		_delay_ms(10);
		for (int j = 0; j < 2; j++) {
			enc_reset(j);
		}
		_delay_ms(10);
		float rps[2];
		for (int j = 0; j < 2; j++) {
			rps[j] = (enc_get(j)/2500.0f)/.01;
		}
		printf("%f %f %u\n", (double)rps[0], (double)rps[1], i);

	}
	motor_off(0);
	motor_off(1);
}

static void caltest() {
	printf("Motor Calibration > \n");
	printf("Choose motor 0 or 1 > ");
	uint16_t motor;
	scanf("%d", &motor);
	printf("%u\n", motor);
	printf("Choose time > ");
	uint16_t time;
	scanf("%d", &time);
	printf("%u\n", time);
	enc_reset(motor);
	motor_setpwm(motor, 700);
	for (unsigned int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motor_off(motor);
	_delay_ms(1000);
	uint16_t steps = enc_get(motor);
	printf("%u\n", steps);
}

static void rapidtest() {
	printf("Motor Calibration > \n");
	printf("Choose motor 0 or 1 > ");
	uint16_t motor;
	scanf("%d", &motor);
	printf("%u\n", motor);
	printf("Choose time > ");
	uint16_t time;
	scanf("%d", &time);
	printf("%u\n", time);
	enc_reset(motor);
	motor_setpwm(motor, 700);
	for (unsigned int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motor_off(motor);
	_delay_ms(1000);
	uint16_t steps = enc_get(motor);
	printf("%u\n", steps);
}
