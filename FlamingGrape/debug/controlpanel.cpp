#include "debug/controlpanel.h"
#include "debug/debug.h"
#include "debug/tests.h"
#include "control/linefollow.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "control/deploy.h"
#include "competition/navfast.h"
#include "competition/navdeploy.h"
#include "hw/motor.h"
#include "hw/mag.h"
#include "hw/enc.h"
#include "hw/adc.h"
#include "util.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

void controlpanel_init() {
	printf_P(PSTR("Starting up\n"));
}

void controlpanel() {
	while (true) {
		switch (controlpanel_promptChar("Main")) {
			case 'd':
				controlpanel_drive();
				break;
			case 'm':
				controlpanel_motor();
				break;
			case 's':
				controlpanel_sensor();
				break;
			case 't':
				controlpanel_tests();
				break;
			case 'D':
				controlpanel_deploy();
				break;
			case 'q':
				printf_P(PSTR("Quitting...\n"));
				return;
			default:
				printf_P(PSTR("Unknown. Commands: drive, sensors, tests\n"));
				break;
		}
	}
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
				
			case 'W':
				drive_fd_dist(speed, 16);
				break;
			case 'A':
				drive_lturn_deg(speed, 90);
				break;
			case 'S':
				drive_bk_dist(speed, 16);
				break;
			case 'D':
				drive_rturn_deg(speed, 90);
				break;	
			case '=':
				speed += 2;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '-':
				speed -= 2;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '+':
				speed += 10;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '_':
				speed -= 10;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
				
			case 'g': {
				PIDGains newgains;
				if (controlpanel_promptGains("motorcontrol", motorcontrol_getGains(), newgains)) {
					motorcontrol_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Canceled.\n"));
				}
				break;
			}
			
			case 'p':
				motorcontrol_setDebug(false);
				printf_P(PSTR("Debug disabled\n"));
				break;
				
			case 'c':
				motorcontrol_setEnabled(false);
				printf_P(PSTR("Motor control disabled\n"));
				break;
				
			case 'P':
				motorcontrol_setDebug(true);
				break;
				
			case 'q':	
				motorcontrol_setEnabled(false);
				return;

			case 'z':
				speed = 40;
				for (int i = 0; i < 20; i++) {
					drive_fd(speed);
					_delay_ms(25);
					drive_rturn(speed);
					_delay_ms(50);
					drive_fd(speed);
					_delay_ms(25);
					drive_lturn(speed);
					_delay_ms(50);
				}
				drive_stop();
				speed = 20;
				break;

			case 'Z':
				speed = 100;
				for (int i = 0; i < 20; i++) {
					drive_bk(speed);
					_delay_ms(25);
					drive_rturn(speed);
					_delay_ms(50);
					drive_bk(speed);
					_delay_ms(25);
					drive_lturn(speed);
					_delay_ms(50);
				}
				speed = 20;
				break;

			default:
				motorcontrol_setEnabled(false);
				printf_P(PSTR("Unknown. Commands: WASD, +-\n"));
				break;
		}
	}
}

void controlpanel_motor() {
	bool motenables[4] = {0, 0, 0, 0};
	int16_t pwm = 0;
	
	while (true) {	
		char ch = controlpanel_promptChar("Motor");
		if (ch >= '0' && ch <= '3') {
			int num = ch-'0';
			bool &enable = motenables[num];
			enable = !enable;
			printf_P(PSTR("Motor %d %s\n"), num, enable ? "enabled" : "disabled");
		} else if (ch == 'e') {
			printf_P(PSTR("L %i R %i\n"), enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
		} else if (ch == 'E') {
			printf_P(PSTR("Encoders reset\n"));
			enc_reset(MOTOR_LEFT);
			enc_reset(MOTOR_RIGHT);
		} else {
			switch (ch) {
				case 'x':
					pwm += 50;
					break;
					
				case 'z':
					pwm -= 50;
					break;
					
				case 'X':
					pwm += 200;
					break;
					
				case 'Z':
					pwm -= 200;
					break;	
					
				case 'a':
					pwm = -motor_maxpwm;
					break;
					
				case 's':
					pwm = motor_maxpwm;
					break;
					
				case 'd':
					pwm = -pwm;
					break;
					
				case ' ':
					pwm = 0;
					break;
					
				case 'q':
					motor_allOff();
					return;
					
				default:
					printf_P(PSTR("Unknown. Commands 0-3 enable motors, zx pwm, a min, s max\n")); 
					break;
			}
			
			if (pwm > motor_maxpwm)
				pwm = motor_maxpwm;
			else if (pwm < -motor_maxpwm)
				pwm = -motor_maxpwm;
				
			printf_P(PSTR("PWM %d\n"), pwm);
		}
							
		for (uint8_t i=0; i<4; i++)
			motor_setpwm(i, motenables[i] ? pwm : 0);
	}
}

void controlpanel_sensor() {
	while (true) {
		switch (controlpanel_promptChar("Sensor")) {		
			case 'a':
				for (int i=0; i<8; i++)
					printf_P(PSTR("%d "), adc_sampleAverage(i, 10));
				putchar('\n');
				break;

			case 'r':
				while(true) {
					switch(controlpanel_promptChar("Rangefinder")) {
						case 'q':
							printf_P(PSTR("Front Left Range: %f\n"), adc_sampleRangeFinder(ADC_FRONT_LEFT_RANGE));
							break;
						case 'e':
							printf_P(PSTR("Front Right Range: %f\n"), adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE));
							break;
						case 'a':
							printf_P(PSTR("Side Left Range: %f\n"), adc_sampleRangeFinder(ADC_SIDE_LEFT_RANGE));
							break;
						case 'd':
							printf_P(PSTR("Side Right Range: %f\n"), adc_sampleRangeFinder(ADC_SIDE_RIGHT_RANGE));
							break;
						case 'z':
							return;
						default:
							printf_P(PSTR("q - front left, e - front right, a - side left, d - side right, z - exit\n"));
							break;
					}
				}
				break;

			case 'l': {
				uint16_t linebuf[linesensor_count];
				linesensor_read(linebuf);
				for (int i=0; i<linesensor_count; i++)
					printf_P(PSTR("%-5u "), linebuf[i]);
				putchar('\n');
				break;
			}
			
			case 'L': {
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
				break;
			}
			
			case 'b':
				printf_P(PSTR("Battery voltage: %.2f\n"), adc_getBattery());
				break;

			case 'm': {
				MagReading reading = mag_getReading();
				printf_P(PSTR("mag: %5d %5d %5d\n"), reading.x, reading.y, reading.z);
				break;
			}

			case 'M': {
				MagReading first_reading = mag_getReading();
				MagReading reading;
				bool returning = false;
				while (true) {
					printf_P(PSTR("%5d %5d %5d\n"), reading.x, reading.y, reading.z);
					drive_lturn(10);
					reading = mag_getReading();
					if (!returning && (abs(reading.x - first_reading.x) > 100)) {
						returning = true;
					} else if (returning && (abs(reading.x - first_reading.x) < 20)) {
						break;
					}
				}
				drive_stop();
				break;
			}
			
			case 'q':
				return;
				
			default:
				printf_P(PSTR("Unknown. Commands: linesensor-raw, Linesensor-full, analog dump, battery\n"));
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
				if (!controlpanel_prompt("Velocity", "%f", &vel)) {
					printf_P(PSTR("Canceled.\n"));
					break;
				}
					
				printf_P(PSTR("Push any key to stop. "));
				linefollow_start(vel);
				getchar();
				linefollow_stop();
				putchar('\n');
				
				printf_P(PSTR("Turn:\t")); linefollow_printTurn(linefollow_getLastTurn()); putchar('\n');
				printf_P(PSTR("Feat:\t")); linefollow_printFeature(linefollow_getLastFeature()); putchar('\n');
				break;
			}
			
			case 'i': {
				while (true) {
					linefollow_start(60);
					linefollow_waitDone();
					
					if (linefollow_getLastFeature() == FEATURE_INTERSECTION)
						break;
					else if (linefollow_getLastTurn() == TURN_LEFT)
						drive_lturn_deg(50, 80);
					else if (linefollow_getLastTurn() == TURN_RIGHT)
						drive_rturn_deg(50, 80);
					else
						break;
				}
				drive_stop();
				
				if (linefollow_getLastFeature() == FEATURE_INTERSECTION)
					printf_P(PSTR("Intersection!\n"));
				else
					printf_P(PSTR("Error\n"));
				break;
			};
			
			case 'l': {
				bool ok = navfast_loopback();
				printf_P(PSTR("Loopback ok: %d\n"), ok);
				if (ok) {
					static bool right=false;
					ok = navfast_leftright(right);
					printf_P(PSTR("leftirght ok: %d\n"), ok);
					right = !right;
				}
				break;
			}
			
			case 'd': {
				navdeploy_loopback();
				navdeploy_deploy();
				navdeploy_aroundBox();
				navdeploy_middle();
				navdeploy_deploy();
				navdeploy_aroundBox();
				break;
			}
				
			case 'g': {
				PIDGains newgains;
				if (controlpanel_promptGains("linefollow", linefollow_getGains(), newgains)) {
					linefollow_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Canceled.\n"));
				}
				break;
			}
			
			case 'm':
				tests_pwm();
				break;
				
			case 'P':
				linefollow_setDebug(true);
				break;
				
			case 'p':
				linefollow_setDebug(false);
				break;

			case 'h':
				motor_setpwm(MOTOR_DEPLOY, 700);
				while (adc_sampleAverage(ADC_BEAM_BREAK, 5) < 3500) { }
				printf("Entered\n");
				_delay_ms(100);
				while (adc_sampleAverage(ADC_BEAM_BREAK, 5) > 3500) { }
				printf("Left\n");
				motor_setpwm(MOTOR_DEPLOY, 0);
				
				_delay_ms(2000);
				
			
				drive_fd_dist(50, 30);
				motor_setpwm(MOTOR_DEPLOY, 700);
				_delay_ms(1500);
				motor_setpwm(MOTOR_DEPLOY, motor_maxpwm);
				_delay_ms(1500);
				motor_setpwm(MOTOR_DEPLOY, 700);
				drive_bk_dist(4, 5);
				motor_setpwm(MOTOR_DEPLOY, 0);
				break;

			case 'L':
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
				break;
				
			case 'q':
				return;

			default:
				printf_P(PSTR("Unknown. Commands: follow line, gain set, pwm test\n"));
				break;
		}
	}
}

void controlpanel_deploy() {
	while (true) {
		switch (controlpanel_promptChar("Deploy")) {
			case 'o':
				deploy_out();
				break;
			case 'O':
				deploy_out(true);
				break;
			case 'i':
				deploy_in();
				break;
			case 'I':
				deploy_in(true);
				break;
			case 'b':
				printf("Beambreak: %d\n", deploy_getBeamBreak());
				break;
			case 'd':
				deploy_start();
				break;
			case ' ':
				deploy_stop();
				break;
			case 'q':
				deploy_stop();
				return;
		}
	}
}	

int controlpanel_prompt(const char *prompt, const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	
	printf_P(PSTR("%s# "), prompt);
	
	char buf[32];
	fgets(buf, sizeof(buf), stdin);
	return vsscanf(buf, fmt, argp);
}

char controlpanel_promptChar(const char *prompt) {
	printf_P(PSTR("%s> "), prompt);
	
	char ch = getchar();
	putchar('\n');
	return ch;
}

bool controlpanel_promptGains(const char *name, const PIDGains &curgains, PIDGains &gains) {
	printf_P(PSTR("Setting gains for %s\n"), name);
	printf_P(PSTR("Current gains: P %.4f I %.4f D %.4f MaxI %.4f\n"), curgains.p, curgains.i, curgains.d, curgains.maxi);
	
	if (controlpanel_prompt("P", "%f", &gains.p) != 1)
		gains.p = curgains.p;
	if (controlpanel_prompt("I", "%f", &gains.i) != 1)
		gains.i = curgains.i;
	if (controlpanel_prompt("D", "%f", &gains.d) != 1)
		gains.d = curgains.d;
	if (controlpanel_prompt("MaxI", "%f", &gains.maxi) != 1)
		gains.maxi = curgains.maxi;
		
	printf_P(PSTR("New gains: P %.4f I %.4f D %.4f MaxI %.4f\n"), gains.p, gains.i, gains.d, gains.maxi);
	return controlpanel_promptChar("Ok? [y/n]") == 'y';
}
