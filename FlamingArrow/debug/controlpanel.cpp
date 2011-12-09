#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>

#include "control/linefollow.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include "hw/temp.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "hw/sensor.h"

#include "debug/controlpanel.h"

void controlpanel_init() {
	printf("Starting up\n");
}

void controlpanel_motor() {
	while (true) {
		printf("Motor > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'w':
				motor_setpwm(0, 1024);
				motor_setpwm(1, 1024);
				_delay_ms(50);
				motor_setpwm(0, 0);
				motor_setpwm(1, 0);
				break;
			case 'a':
				motor_setpwm(0, -1024);
				motor_setpwm(1, 1024);
				_delay_ms(50);
				motor_setpwm(0, 0);
				motor_setpwm(1, 0);
				break;
			case 's':
				motor_setpwm(0, -1024);
				motor_setpwm(1, -1024);
				_delay_ms(50);
				motor_setpwm(0, 0);
				motor_setpwm(1, 0);
				break;
			case 'd':
				motor_setpwm(0, 1024);
				motor_setpwm(1, -1024);
				_delay_ms(50);
				motor_setpwm(0, 0);
				motor_setpwm(1, 0);
				break;
			case 'f':
				motorcontrol_setrps(0, 0);
				motorcontrol_setrps(1, 0);
				motorcontrol_setDebug(true);
				_delay_ms(1000);
				{ float t = 0;
				while(true) {
					t += .01;
					motorcontrol_setrps(0, 2+sin(t*3));
					_delay_ms(10);
				} }
				getchar();
				motorcontrol_setDebug(false);
				motorcontrol_disable(0);
				motorcontrol_disable(1);
				break;
			case 'e':
				printf("%i %i\n", enc_get(0), enc_get(1));
				break;
			case 'b':
				return;
			case 't':
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
				break;
			case 'c':
				{
					printf("Motor Calibration > \n");
					bool done = false;
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
					for (int i = 0; i < time; i++) {
						_delay_ms(1);
					}
					motor_off(motor);
					_delay_ms(1000);
					uint16_t steps = enc_get(motor);
					printf("%u\n", steps);
				}
				break;
			case 'r':			// Rapid speed changes
				{
					uint16_t i = 0;
					motorcontrol_setDebug(true);
					_delay_ms(2000);
					while(i < 4) {
						printf("%u\n", i);
						motorcontrol_setvel(0, i);
						motorcontrol_setvel(1, i);
						i = i + 1;
						_delay_ms(1000);
					}

					motorcontrol_setvel(0, 0.001);
					motorcontrol_setvel(1, 0.001);
					_delay_ms(1000);
					printf("vel = 1");
					motorcontrol_setvel(0, 4);
					motorcontrol_setvel(1, 4);
					_delay_ms(1000);
					printf("vel = 4");
					motorcontrol_setvel(0, 0.001);
					motorcontrol_setvel(1, 0.001);
					_delay_ms(1000);
					motorcontrol_setvel(0, 4);
					motorcontrol_setvel(1, 4);
					_delay_ms(1000);
					motorcontrol_setvel(0, 0.001);
					motorcontrol_setvel(1, 0.001);
					_delay_ms(1000);
					motorcontrol_setvel(0, 4);
					motorcontrol_setvel(1, 4);
					_delay_ms(1000);
					motorcontrol_setvel(0, 0);
					motorcontrol_setvel(1, 0);
					motorcontrol_setDebug(false);
					printf("done");
				}
				break;
			case 'h':
				drive_fwd(30, 1);
			case 'i':
				drive_bck(30, 15);
				break;
			case 'j':
				drive_fwd(30, 10);
				break;
			case 'k':
				drive_fwd(30, 20);
				break;
			case 'l':
				drive_fwd(30, 30);
				break;
			case 'm':
				controlpanel_move_setspeed();
				break;
			default:
				printf("Unknown. Commands: WASD, Forward, Encoders, Back.\n");
				break;
		}
	}
}

void controlpanel_move_setspeed() {
	while(true) {
		printf("f: fast, d: medium, s: slow > ");
		char ch = getchar();
		int32_t speed = 0;
		printf("%c\n", ch);
		switch (ch) {
			case 'f':
				speed = 40;
				break;
			case 'd':
				speed = 25;
				break;
			case 's':
				speed = 10;
				break;
			case 'b':
				return;
			default:
				printf("Unknown. Commands: Fast, meDium, Slow, Back.\n");
				break;
		}
		controlpanel_move(speed);
	}
}

void controlpanel_move(int32_t speed) {
	while (true) {
		printf("WASD: > ");
		char ch = getchar();
		uint16_t default_dist = 5;
		uint16_t default_turn = 20;
		printf("%c\n", ch);
		switch (ch) {
			case 'w':
				drive_fwd(default_dist, speed);
				break;
			case 'a':
				drive_l_turn(default_turn, speed);
				break;
			case 's':
				drive_bck(default_dist, speed);
				break;
			case 'd':
				drive_r_turn(default_turn, speed);
				break;
			case 'b':
				return;
			default:
				printf("WASD controls, b for back: \n");
				break;
		}
	}
}

void controlpanel_temp() {
	while (true) {
		printf("Temperature > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'a':
				printf("Ambient: %f\n", (double)temp_get(AMB));
				break;
			case 'o':
				printf("Object: %f\n", (double)temp_get(OBJ));
				break;
			case 'b':
				return;
			default:
				printf("Unknown. Commands: Ambient, Object, Back.\n");
				break;
		}
	}
}

static bool printLineSensorUpdates = false;

void controlpanel_lineSensorUpdate(const uint16_t *readings) {
	if(printLineSensorUpdates)
		printf("LS %u %u %u %u %u %u %u %u\n",
			readings[0],
			readings[1],
			readings[2],
			readings[3],
			readings[4],
			readings[5],
			readings[6],
			readings[7]);
}

void controlpanel_linesensor() {
	while (true) {
		printf("Line Sensor > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'f':
				linefollow_setEnabled(true);
				break;
			case 's':
				linefollow_setEnabled(false);
				break;
			case 'p':
				printLineSensorUpdates = true;
                                getchar();
				printLineSensorUpdates = false;
				break;
			case 'b':
				return;
			default:
				printf("Unknown. Commands: Follow, Stop following, Print updates, Back.\n");
				break;
		}
	}
}

#include "hw/adc.h"

void controlpanel_sensor() {
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

			case 't':
				for (int i=0; i<8; i++)
					printf("%d ", adc_sample(i));
				break;

			case 'b':
				return;
		}
	}
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
			case 't':
				controlpanel_temp();
				break;
			case 'l':
				controlpanel_linesensor();
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
