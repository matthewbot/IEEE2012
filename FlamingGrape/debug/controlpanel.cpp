#include "debug/controlpanel.h"
#include "debug/debug.h"
#include "debug/tests.h"
#include "control/linefollow.h"
#include "control/magfollow.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "control/deploy.h"
#include "competition/nav.h"
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

static const char unknown_str[] PROGMEM = "Unknown. ? for help.";

void controlpanel() {
	while (true) {
		switch (controlpanel_promptChar("Main")) {
			case 'd':
				controlpanel_drive();
				break;
			case 'g':
				controlpanel_gains();
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
			case 'n':
				controlpanel_nav();
				break;
			case 'D':
				controlpanel_deploy();
				break;
			default:
				puts_P(unknown_str);
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  d - Drive\n"
					"  D - Deploy\n"
					"  g - Gains\n"
					"  m - Motor\n"
					"  s - Sensor\n"
					"  n - Nav\n"
					"  t - Tests";
				puts_P(msg);
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
				puts_P(unknown_str);
				drive_stop();
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Drive commands:\n"
					"  wasd  - Control robot\n"
					"  space - Stop\n"
					"  -=_+  - Adjust speed\n"
					"  WASD  - Execute distance moves\n"
					"  c	 - Disable motor control\n"
					"  Pp	- Enable/Disable motor control debug\n"
					"  zZ	- Moonwalk (WIP)\n"
					"  q	 - Back";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_gains() {
	PIDGains newgains;
	while (true) {
		char ch = controlpanel_promptChar("Gains");
		switch (ch) {
			case 'm':
				if (controlpanel_promptGains("motorcontrol", motorcontrol_getGains(), newgains)) {
					motorcontrol_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 'M':
				if (controlpanel_promptGains("magfollow", magfollow_getGains(), newgains)) {
					magfollow_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 'l':
				if (controlpanel_promptGains("linefollow", linefollow_getGains(), newgains)) {
					linefollow_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Gain commands:\n"
					"  m - Adjust Motor Gains\n"
					"  M - Adjust Magfollow Gains\n"
					"  l - Adjust Linefollow Gains\n"
					"  q	 - Back\n";
				puts_P(msg);
				break;
			default:
				printf_P(PSTR("Unknown Command, type '?' for help.\n"));
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
					puts_P(unknown_str);
					break;
					
				case '?':
					static const char msg[] PROGMEM = 
						"Motor commands:\n"
						"  0123  - Toggle motors\n"
						"  zxZX  - Adjust PWM\n"
						"  d	 - Flip PWM direction\n"
						"  space - Zero PWM\n"
						"  as	 - Max PWM\n"
						"  e     - Display encoders\n"
						"  q	 - Back";
					puts_P(msg);
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
						case 'w':
							printf_P(PSTR("Front Left Range: %f\n"), adc_sampleRangeFinder(ADC_FRONT_LEFT_RANGE));
							break;
						case 'e':
							printf_P(PSTR("Front Right Range: %f\n"), adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE));
							break;
						case 's':
							printf_P(PSTR("Side Left Range: %f\n"), adc_sampleRangeFinder(ADC_SIDE_LEFT_RANGE));
							break;
						case 'd':
							printf_P(PSTR("Side Right Range: %f\n"), adc_sampleRangeFinder(ADC_SIDE_RIGHT_RANGE));
							break;
						case 'q':
							return;
						default:
							printf_P(PSTR("w - front left, e - front right, s - side left, d - side right, q - exit\n"));
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
				float heading = magfollow_getHeading();
				heading = radtodeg(heading);
				printf_P(PSTR("Mag Heading: %f\n"), heading);
				break;
			}

			case 'H': {
				float newheading;
				controlpanel_prompt("Heading", "%f", &newheading);
				magfollow_setHeading(degtorad(newheading));
				break;
			}

			case 'q':
				return;
				
			default:
				puts_P(unknown_str);
				break;
				
			case '?':
				static const char msg[] PROGMEM = 
					"Sensor commands\n"
					"  a - Dump analog port A\n"
					"  r - Rangefinder control panel\n" // Will if you get a chance merge these into the main control panel, one command to show all four rangefinders
					"  l - Raw line sensor readings\n"
					"  L - Processed line sensor readings\n"
					"  b - Battery voltage (approx)\n"
					"  m - Magnetometer\n"
					"  M - Magnetometer Heading\n"
					"  H - Set Magnetometer Heading\n"
					"  q - Back";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_nav() {
	while (true) {
		switch (controlpanel_promptChar("Nav")) {
			case 'i':
				if (nav_linefollowIntersection())
					printf_P(PSTR("Intersection!\n"));
				else
					printf_P(PSTR("Error\n"));
				break;
				
			case 't': {
				int turns;
				if (!controlpanel_prompt("Turns", "%d", &turns)) {
					printf_P(PSTR("Canceled.\n"));
					break;
				}
				
				if (nav_linefollowTurns(turns))
					printf_P(PSTR("Ok.\n"));
				else
					printf_P(PSTR("Error\n"));
				break;
			}
			
			case 'g': {
				float heading;
				if (!controlpanel_prompt("Heading", "%f", &heading)) {
					printf_P(PSTR("Canceled.\n"));
					break;
				}
				
				float dist;
				if (!controlpanel_prompt("Dist", "%f", &dist)) {
					printf_P(PSTR("Canceled.\n"));
					break;
				}
				
				nav_magGo(heading, dist);
				break;
			}
			
			case 'f':
				navfast_lap();
				break;
			
			case 'd':
				navdeploy_lap();
				break;

			case 'q':
				return;
				
			default:
				puts_P(unknown_str);
				break;
				
			case '?':
				static const char msg[] PROGMEM = 
					"Nav commands\n"
					"  i - Intersection line follow\n"
					"  t - Turn counting line follow\n"
					"  g - magGo along heading for specific distance\n"
					"  f - run a fast lap\n"
					"  d - run a deploying lap\n"
					"  q - back";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_tests() {
	while (true) {
		switch (controlpanel_promptChar("Tests")) {
			case 'f':
				tests_linefollow();
				break;
			
			case 'p':
				tests_pwm();
				break;
				
			case 'M':
				tests_mag();
				break;
				
			case 'm':
				tests_magfollow();
				break;

			case 'D':
				linefollow_setDebug(true);
				break;
				
			case 'd':
				linefollow_setDebug(false);
				break;

			case 'L':
				tests_led();
				break;

			case 'l':
				tests_movingLineRead();
				break;
				
			case 'q':
				return;

			default:
				puts_P(unknown_str);
				break;
				
			case '?':
				static const char msg[] PROGMEM = 
					"Test commands\n"
					"  f  - Linefollow test\n"
					"  p  - Test motor pwm range (floors motors)\n"
					"  M  - Test magnetometer (spins in place)\n"
					"  m  - Magfollow test\n"
					"  Dd - Enables/Disable line follow debugging\n"
					"  L  - Tests debug LEDs\n"
					"  l  - Runs motors while printing linesensor data\n"
					"  q  - Back";
				puts_P(msg);
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
			default:
				puts_P(unknown_str);
				static const char msg[] PROGMEM = 
					"Deploy commands\n"
					"  io    - Deploy in/out\n"
					"  IO    - Deploy in/out fast\n"
					"  b     - Read break beam\n"
					"  d     - Put module to deploy position\n"
					"  space - Stop\n"
					"  q     - Back";
				puts_P(msg);
				break;
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
