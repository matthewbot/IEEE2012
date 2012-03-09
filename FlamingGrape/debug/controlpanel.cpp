#include "debug/controlpanel.h"
#include "debug/debug.h"
#include "debug/tests.h"
#include "control/linefollow.h"
#include "control/magfollow.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "control/deploy.h"
#include "control/traj.h"
#include "competition/nav.h"
#include "competition/navfast.h"
#include "competition/navdeploy.h"
#include "competition/sensorcomms.h"
#include "hw/motor.h"
#include "hw/mag.h"
#include "hw/enc.h"
#include "hw/adc.h"
#include "hw/tick.h"
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
			case 'c':
				controlpanel_sensorcomms();
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
			case 'x':
				drive_stop(DM_TRAJ);
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
				drive_fdDist(speed, 50);
				break;
			case 'A':
				drive_lturnDeg(speed, 90);
				break;
			case 'S':
				drive_bkDist(speed, 50);
				break;
			case 'D':
				drive_rturnDeg(speed, 90);
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

			case 'z':			// Does moonwalk forwards
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

			case 'Z':			// Does moonwalk backwards
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
				
			case 'm': {
				float amax = drive_getTrajAmax();
				printf_P(PSTR("Current amax: %.2f\n"), amax);
				if (controlpanel_prompt("amax", "%f", &amax) != 1) {
					printf_P(PSTR("Cancelled.\n"));
					continue;
				}
				
				drive_setTrajAmax(amax);
				printf_P(PSTR("Amax set to: %.2f\n"), amax);
				break;
			}

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
					"  Pp	 - Enable/Disable motor control debug\n"
					"  zZ	 - Moonwalk (WIP)\n"
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
	int16_t PWM = 0;
	
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
					PWM += 50;
					break;
					
				case 'z':
					PWM -= 50;
					break;
					
				case 'X':
					PWM += 200;
					break;
					
				case 'Z':
					PWM -= 200;
					break;	
					
				case 'a':
					PWM = -motor_maxPWM;
					break;
					
				case 's':
					PWM = motor_maxPWM;
					break;
					
				case 'd':
					PWM = -PWM;
					break;
					
				case ' ':
					PWM = 0;
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
			
			if (PWM > motor_maxPWM)
				PWM = motor_maxPWM;
			else if (PWM < -motor_maxPWM)
				PWM = -motor_maxPWM;
				
			printf_P(PSTR("PWM %d\n"), PWM);
		}
							
		for (uint8_t i=0; i<4; i++)
			motor_setPWM(i, motenables[i] ? PWM : 0);
	}
}

void controlpanel_sensor() {
	while (true) {
		switch (controlpanel_promptChar("Sensor")) {		
			case 'a':			// Prints raw adc sensor values pins 0 - 7
				for (int i=0; i<8; i++)
					printf_P(PSTR("%4d "), adc_sampleAverage(i, 10));
				putchar('\n');
				break;

			case 'r':			// Prints all rangefinder readings in centimeters
				while(true) {
					printf_P(PSTR("Side Left: %5f, Front Left: %5f, Front Right: %5f, Side Right: %5f\n"), adc_sampleRangeFinder(ADC_SIDE_LEFT_RANGE), adc_sampleRangeFinder(ADC_FRONT_LEFT_RANGE), adc_sampleRangeFinder(ADC_FRONT_RIGHT_RANGE), adc_sampleRangeFinder(ADC_SIDE_RIGHT_RANGE));
				}
				break;

			case 'l': {			// Prints out raw linesensor data
				uint16_t linebuf[linesensor_count];
				linesensor_read(linebuf);
				for (int i=0; i<linesensor_count; i++)
					printf_P(PSTR("%-5u "), linebuf[i]);
				putchar('\n');
				break;
			}
			
			case 'L': {			// Prints out full crunched linesensor data
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
				printf_P(PSTR("Time:\t%uus\n"), time);
				break;
			}
			
			case 't': {
				float thresh;
				printf_P(PSTR("Current threshold: %f\n"), linefollow_getThresh()); 
				if (controlpanel_prompt("Threshold", "%f", &thresh) == 1) {
					printf_P(PSTR("Threshold changed to %f\n"), thresh);
					linefollow_setThresh(thresh);
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			}
			
			case 'b':
				printf_P(PSTR("Battery voltage: %.2f\n"), adc_getBattery());
				break;

			case 'm': {			// Prints out raw magnetometer data
				MagReading reading = mag_getReading();
				printf_P(PSTR("mag: %5d %5d %5d\n"), reading.x, reading.y, reading.z);
				break;
			}

			case 'M': {			// Prints out magnetometer calibrated heading
				float heading = magfollow_getHeading();
				heading = radtodeg(heading);
				printf_P(PSTR("Mag Heading: %f\n"), heading);
				break;
			}

			case 'H': {			// Sets current heading of robot to prompted heading from user
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
					"  r - Rangefinder control panel\n"
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
			case 'i':			// Linefollows until an intersection
				if (nav_linefollowIntersection())
					printf_P(PSTR("Intersection!\n"));
				else
					printf_P(PSTR("Error\n"));
				break;
				
			case 't': {			// Linefollows for a user prompted number of turns
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
			
			case 'g': {			// Does a NavGo at a user prompted magnetometer heading and for a user prompted distance
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
			case 'f':		// Linefollow at user prompted velocity until key press
				tests_linefollow();
				break;
			
			case 'p':		// Ramps motors forwards to full speed then backwards to full speed
				tests_PWM();
				break;
				
			case 'M':		// Spins in place while printing raw magnetometer data for 1 rev
				tests_mag();
				break;
				
			case 'm':		// Follows a user prompted heading at prompted speed of magnetometer
				tests_magfollow();
				break;

			case 'D':		// Turns on debug printing when linefollowing
				linefollow_setDebug(true);
				break;
				
			case 'd':		// Turns off debug printing when linefollowing
				linefollow_setDebug(false);
				break;

			case 'e':		// Blinks through debug LEDs
				tests_led();
				break;

			case 'L':		// Prints minimum of each sensor on linesensor array over 4 seconds of readings
				uint16_t min[8];
				tests_linesensorMin(min);
				for (int i = 0; i < 8; i++) {
					printf_P(PSTR("%5u "), min[i]);
				}
				printf_P(PSTR("\n"));
				break;

			case 'l':		// Prints out linesensor data while spinning wheels for encoder interference
				tests_movingLineRead();
				break;
				
			case 'q':
				return;

			case 't':
				printf_P(PSTR("Tick Length %u us\n"), tick_getLength());
				break;

			default:
				puts_P(unknown_str);
				break;
				
			case '?':
				static const char msg[] PROGMEM = 
					"Test commands\n"
					"  f  - Linefollow test\n"
					"  p  - Test motor PWM range (floors motors)\n"
					"  M  - Test magnetometer (spins in place)\n"
					"  m  - Magfollow test\n"
					"  Dd - Enables/Disable line follow debugging\n"
					"  e  - Tests debug LEDs\n"
					"  L  - Prints min val of each linesensor over 4 secs\n"
					"  l  - Runs motors while printing linesensor data\n"
					"  t  - Prints encoder ticks\n"
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

void controlpanel_sensorcomms() {
	while (true) {
		switch (controlpanel_promptChar("Comms")) {
			case 'c':
				printf_P(PSTR("Board count: %d\n"), sensorcomms_getOnlineBoardCount());
				break;
				
			case 's': {
				static const char symbols[] = {'V', 'C', 'T', 'S'};
				for (int board=0; board < BOARDNUM_MAX; board++) {
					printf_P(PSTR("%c "), symbols[board]);
					sensorcomms_printBoardStatus(sensorcomms_getBoardStatus((BoardNum)board));
					putchar('\n');
				}
				break;
			}
				
			case 'u': {
				int board;
				if (controlpanel_prompt("Board", "%d", &board) != 1) {
					printf_P(PSTR("Cancelled.\n"));
					break;
				}
				
				sensorcomms_updateBoard((BoardNum)board);
				printf_P(PSTR("Updating board %d\n"), board);
				break;
			}
			
			case 'U': {
				for (int board=0; board < sensorcomms_getOnlineBoardCount(); board++) {
					sensorcomms_updateBoard((BoardNum)board);
					printf_P(PSTR("Updating board %d\n"), board);
					sensorcomms_waitBoard((BoardNum)board);
				}
				printf_P(PSTR("Done!\n"));
				
				break;
			}
			
			case 'R':
				if (controlpanel_promptChar("Really reset comms? [y/n]") == 'y') {
					printf_P(PSTR("Comms reset\n"));
					sensorcomms_reset();
				}
				break;
				
			case 'd': {
				uint8_t buf[10];
				for (int board=0; board < BOARDNUM_MAX; board++) {
					sensorcomms_getBoardReading(buf, sizeof(buf), (BoardNum)board);
					for (int i=0; i<sizeof(buf); i++)
						printf_P(PSTR("%02x "), buf[i]);
					if (sensorcomms_getBoardReadingValid((BoardNum)board))
						printf_P(PSTR(" - valid\n"));
					else
						printf_P(PSTR(" - invalid\n"));
				}
				break;
			}
			
			case 'q':
				return;
				
			// TODO help menu
				
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
