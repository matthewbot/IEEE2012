#include "debug/controlpanel.h"
#include "debug/debug.h"
#include "debug/tests.h"
#include "control/linefollow.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "competition/nav.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include "hw/adc.h"
#include "util.h"
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

void controlpanel_init() {
	printf("Starting up\n");
}

void controlpanel() {
	while (true) {
		switch (controlpanel_promptChar("Main")) {
			case 'd':
				controlpanel_drive();
				break;
			case 's':
				controlpanel_sensor();
				break;
			case 't':
				controlpanel_tests();
				break;
			case 'q':
				printf("Quitting...\n");
				return;
			default:
				printf("Unknown. Commands: drive, sensors, tests\n");
				break;
		}
	}
}

static void pwm(int mot, int16_t pwm) {
	motorcontrol_setEnabled(false);
	motor_setpwm(mot, pwm);
}

void controlpanel_drive() {
	float speed=20;
	while (true) {
		char ch=controlpanel_promptChar("Drive");
		
		switch (ch) {
			case ' ':
				drive_stop();
				break;
			
			case 'w':
				drive_fd(speed);
				break;
			case 'a':
				drive_lturn(speed);
				break;
			case 's':
				drive_bk(speed);
				break;
			case 'd':
				drive_rturn(speed);
				break;
				
			case 'u':
				pwm(MOTOR_LEFT, motor_maxpwm/2);
				break;
			case 'j':
			case 'J':
				pwm(MOTOR_LEFT, 0);
				break;
			case 'n':
				pwm(MOTOR_LEFT, -motor_maxpwm/2);
				break;
			case 'U':
				pwm(MOTOR_LEFT, motor_maxpwm);
				break;
			case 'N':
				motorcontrol_setEnabled(false);
				motor_setpwm(MOTOR_LEFT, -motor_maxpwm);
				break;
				
			case 'i':
				pwm(MOTOR_RIGHT, motor_maxpwm/2);
				break;
			case 'k':
			case 'K':
				pwm(MOTOR_RIGHT, 0);
				break;
			case 'm':
				pwm(MOTOR_RIGHT, -motor_maxpwm/2);
				break;
			case 'I':
				pwm(MOTOR_RIGHT, motor_maxpwm);
				break;
			case 'M':
				pwm(MOTOR_RIGHT, -motor_maxpwm);
				break;
				
			case 'W':
				drive_fd_dist(16, speed);
				break;
			case 'A':
				drive_lturn_deg(90, speed);
				break;
			case 'S':
				drive_bk_dist(16, speed);
				break;
			case 'D':
				drive_rturn_deg(90, speed);
				break;
				
			case '=':
				speed += 2;
				printf("Speed: %f\n", speed);
				break;
			case '-':
				speed -= 2;
				printf("Speed: %f\n", speed);
				break;
			case '+':
				speed += 10;
				printf("Speed: %f\n", speed);
				break;
			case '_':
				speed -= 10;
				printf("Speed: %f\n", speed);
				break;
				
				
			case 'e':
				printf("L %i R %i\n", enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
				break;
			case 'E':
				printf("Encoders reset\n");
				enc_reset(MOTOR_LEFT);
				enc_reset(MOTOR_RIGHT);
				break;
				
			case 'g': {
				PIDGains newgains;
				if (controlpanel_promptGains("motorcontrol", motorcontrol_getGains(), newgains)) {
					motorcontrol_setGains(newgains);
					printf("Gains set!\n");
				} else {
					printf("Canceled.\n");
				}
				break;
			}
			
			case 'p':
				motorcontrol_setDebug(false);
				printf("Debug disabled\n");
				break;
				
			case 'c':
				motorcontrol_setEnabled(false);
				printf("Motor control disabled\n");
				break;
				
			case 'P':
				motorcontrol_setDebug(true);
				break;
				
			case 'q':	
				motorcontrol_setEnabled(false);
				motor_allOff();
				return;

			case 'z':
				speed = 100;
				for (int i = 0; i < 100; i++) {
					if (i%4 == 0) {
						drive_fd(speed);
					} else if (i%4 == 1) {
						drive_rturn(speed);
					} else if (i%4 == 2) {
						drive_fd(speed);
					} else {
						drive_lturn(speed);
					}
					_delay_ms(50);
				}
				speed = 20;
				break;

			case 'Z':
				//speed = 100;
				for (int i = 0; i < 100; i++) {
					if (i%4 == 0) {
						drive_fd(speed);
					} else if (i%4 == 1) {
						drive_rturn(speed);
					} else if (i%4 == 2) {
						drive_bk(speed);
					} else {
						drive_lturn(speed);
					}
					_delay_ms(50);
				}
				speed = 20;
				break;

			default:
				motorcontrol_setEnabled(false);
				motor_allOff();
				printf("Unknown. Commands: WASD, ujn/ikm, +-, encoders, Encoder clear\n");
				break;
		}
	}
}

void controlpanel_sensor() {
	while (true) {
		switch (controlpanel_promptChar("Sensor")) {		
			case 'a':
				for (int i=0; i<8; i++)
					printf("%d ", adc_sample(i));
				printf("\n");
				break;

			case 'r':
				while(true) {
					switch(controlpanel_promptChar("Rangefinder")) {
						case 'q':
							printf("Front Left Range: %f\n", adc_sampleRangeFinder(ADC_FRONT_LEFT_RANGE));
							break;
						case 'e':
							printf("Front Right Range: %f\n", adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE));
							break;
						case 'a':
							printf("Side Left Range: %f\n", adc_sampleRangeFinder(ADC_SIDE_LEFT_RANGE));
							break;
						case 'd':
							printf("Side Right Range: %f\n", adc_sampleRangeFinder(ADC_SIDE_RIGHT_RANGE));
							break;
						case 'z':
							return;
						default:
							printf("q - front left, e - front right, a - side left, d - side right, z - exit\n");
							break;
					}
				}
				break;

			case 'l': {
				uint16_t linebuf[linesensor_count];
				linesensor_read(linebuf);
				for (int i=0; i<linesensor_count; i++)
					printf("%-5u ", linebuf[i]);
				printf("\n");
				break;
			}
			
			case 'L': {
				debug_resetTimer();
				LineFollowResults results = linefollow_readSensor();
				uint16_t time = debug_getTimer();
				
				printf("Light:\t");
				for (int i=0; i<linesensor_count; i++)
					printf("%2.2f\t", results.light[i]);
				printf("\n");						
				
				printf("Thresh:\t");
				for (int i=0; i<linesensor_count; i++)
					printf("%d\t", results.thresh[i]);
				printf("\n");

				printf("Center:\t%f\n", results.center);
								
				static const char *turnstrs[] = {
					"NONE",
					"LEFT",
					"RIGHT"
				};
				
				printf("Turn:\t%s\n", turnstrs[results.turn]);
				
				static const char *featurestrs[] = {
					"NONE",
					"INTERSECTION",
					"NOLINE"
				};
				printf("Feat:\t%s\n", featurestrs[results.feature]);
				printf("Time:\t%u uS\n", time);
				break;
			}
			
			case 'b':
				printf("Battery voltage: %.2f\n", adc_getBattery());
				break;
			
			case 'q':
				return;
				
			default:
				printf("Unknown. Commands: linesensor-raw, Linesensor-full, analog dump, battery\n");
				break;
		}
	}
}

void controlpanel_tests() {
	bool linedebug=false;
	
	while (true) {
		switch (controlpanel_promptChar("Tests")) {
			case 'f': {
				float vel;
				if (!controlpanel_prompt("Velocity", "%f", &vel))
					printf("Canceled.\n");
					
				printf("Push any key to stop. ");
				linefollow_start(vel, linedebug);
				getchar();
				linefollow_stop();
				putchar('\n');
				
				static const char *turnstrs[] = {
					"NONE",
					"LEFT",
					"RIGHT"
				};
				
				printf("Turn:\t%s\n", turnstrs[linefollow_getLastTurn()]);
				
				static const char *featurestrs[] = {
					"NONE",
					"INTERSECTION",
					"NOLINE"
				};
				
				printf("Last feature: %s\n", featurestrs[linefollow_getLastFeature()]);
				break;
			}
			
			case 'i': {
				while (true) {
					linefollow_start(60);
					linefollow_waitDone();
					
					if (linefollow_getLastFeature() == FEATURE_INTERSECTION)
						break;
					else if (linefollow_getLastTurn() == TURN_LEFT)
						drive_lturn_deg(80, 50);
					else if (linefollow_getLastTurn() == TURN_RIGHT)
						drive_rturn_deg(80, 50);
					else
						break;
				}
				drive_stop();
				
				if (linefollow_getLastFeature() == FEATURE_INTERSECTION)
					printf("Intersection!\n");
				else
					printf("Error\n");
				break;
			};
			
			case 'l': {
				bool ok = nav_loopback();
				printf("Ok: %d\n", ok);
				break;
			}
				
			case 'g': {
				PIDGains newgains;
				if (controlpanel_promptGains("linefollow", linefollow_getGains(), newgains)) {
					linefollow_setGains(newgains);
					printf("Gains set!\n");
				} else {
					printf("Canceled.\n");
				}
				break;
			}
			
			case 'p':
				tests_pwm();
				break;
				
			case 'd':
				linedebug = !linedebug;
				printf("Line follow debugging %s\n", linedebug ? "enabled" : "disabled");
				break;
				
			case 'q':
				return;

			default:
				printf("Unknown. Commands: follow line, gain set, pwm test\n");
				break;
		}
	}
}

int controlpanel_prompt(const char *prompt, const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	
	printf("%s# ", prompt);
	
	char buf[32];
	fgets(buf, sizeof(buf), stdin);
	return vsscanf(buf, fmt, argp);
}

char controlpanel_promptChar(const char *prompt) {
	printf("%s> ", prompt);
	
	char ch = getchar();
	putchar('\n');
	return ch;
}

bool controlpanel_promptGains(const char *name, const PIDGains &curgains, PIDGains &gains) {
	printf("Setting gains for %s\n", name);
	printf("Current gains: P %.4f I %.4f D %.4f MaxI %.4f\n", curgains.p, curgains.i, curgains.d, curgains.maxi);
	
	if (controlpanel_prompt("P", "%f", &gains.p) != 1)
		gains.p = curgains.p;
	if (controlpanel_prompt("I", "%f", &gains.i) != 1)
		gains.i = curgains.i;
	if (controlpanel_prompt("D", "%f", &gains.d) != 1)
		gains.d = curgains.d;
	if (controlpanel_prompt("MaxI", "%f", &gains.maxi) != 1)
		gains.maxi = curgains.maxi;
		
	printf("New gains: P %.4f I %.4f D %.4f MaxI %.4f\n", gains.p, gains.i, gains.d, gains.maxi);
	return controlpanel_promptChar("Ok? [y/n]") == 'y';
}
