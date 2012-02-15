#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>

#include "control/linefollow.h"
#include "hw/motor.h"
#include "hw/enc.h"
#include "hw/temp.h"
#include "hw/adc.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "hw/sensors.h"
#include "util.h"

#include "debug/debug.h"
#include "debug/controlpanel.h"
#include "debug/debug.h"

void controlpanel_init() {
	printf("Starting up\n");
}

void controlpanel() {
	while(true) {
		printf("Main > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'd':
				controlpanel_drive();
				break;
			case 's':
				controlpanel_sensor();
				break;
			case 'a':
				controlpanel_algorithm();
				break;
			case 't':
				controlpanel_tests();
				break;
			case 'q':
				printf("Quitting...\n");
				return;
			case 'x':
				printf("Disabling XBee\n");
				debug_setXBeeEnabled(false);
				break;
			default:
				printf("Unknown. Submenus: Motor, Temp, Line sensor. Commands: Quit.\n");
				break;
		}
	}
}

void controlpanel_drive() {
	float speed=20;
	while (true) {
		printf("Drive > ");
		char ch = getchar();
		printf("%c\n", ch);
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
				motor_setpwm(MOTOR_LEFT, motor_maxpwm/2);
				break;
			case 'j':
				motor_setpwm(MOTOR_LEFT, 0);
				break;
			case 'n':
				motor_setpwm(MOTOR_LEFT, -motor_maxpwm/2);
				break;
			case 'U':
				motor_setpwm(MOTOR_LEFT, motor_maxpwm);
				break;
			case 'J':
				motor_setpwm(MOTOR_LEFT, 0);
				break;
			case 'N':
				motor_setpwm(MOTOR_LEFT, -motor_maxpwm);
				break;
				
			case 'i':
				motor_setpwm(MOTOR_RIGHT, motor_maxpwm/2);
				break;
			case 'k':
				motor_setpwm(MOTOR_RIGHT, 0);
				break;
			case 'm':
				motor_setpwm(MOTOR_RIGHT, -motor_maxpwm/2);
				break;
			case 'I':
				motor_setpwm(MOTOR_RIGHT, motor_maxpwm);
				break;
			case 'K':
				motor_setpwm(MOTOR_RIGHT, 0);
				break;
			case 'M':
				motor_setpwm(MOTOR_RIGHT, -motor_maxpwm);
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
				
			case '+':
				speed += 2;
				printf("Speed: %f\n", (double)speed);
				break;
			case '-':
				speed -= 2;
				printf("Speed: %f\n", (double)speed);
				break;
				
			case 'e':
				printf("L %i R %i\n", enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
				break;
			case 'E':
				printf("Encoders reset\n");
				enc_reset(MOTOR_LEFT);
				enc_reset(MOTOR_RIGHT);
				break;
			case 'q':
				return;

			default:
				printf("Unknown. Commands: WASD, Forward, Encoders, Back.\n");
				break;
		}
	}
	
	motorcontrol_setEnabled(false);
}


//void controlpanel_sensor() { }
//void controlpanel_algorithm() { }

void controlpanel_sensor() {
	uint16_t linebuf[linesensor_count];
	
	while (true) {
		printf("Sensor > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'b':
				printf("Bump: %d\n", sensors_readBump());
				break;
			
			case 'c':
				printf("CapADC: %u\n", sensors_readCapADC());
				break;

			case 'v':
				printf("VoltageADC: %u\n", sensors_readVoltageADC());
				break;

			case 's':
				printf("SignalADC: %u\n", sensors_readSignalADC());
				break;
				
			case 'S': {
				uint16_t data[8];
				memset(data, 0, sizeof(data));
				for (int i=0; i<1000; i++) {
					uint16_t val = sensors_readVoltageADC();
					val <<= 1;
					
					data[val >> 9]++;
				}
				
				for (int i=0; i<8; i++) {
					printf("%u ", data[i]);
				}
				printf("\n");
				
				int leftpos=0;
				while (data[leftpos] < 5)
					leftpos++;
				
				printf("Leftpos: %d\n", leftpos);
					
				int rightpos=7;
				while (data[rightpos] < 5)
					rightpos--;
					
				printf("Rightpos: %d\n", rightpos);
					
				int val = min(data[leftpos], data[rightpos]);
				
				printf("Val: %d\n", val);
				
				bool square=true;
				for (int pos=leftpos+1; pos <= rightpos-1; pos++) {
					printf("%d %d\n", pos, val - (int)data[pos]);
					if (val - (int)data[pos] < 50) {
						square = false;
						break;
					}
				}
				
				printf("Square %d\n", square);
				
				break;
			}
				
			case 'C': {
				printf("Charging cap...\n");
				
				unsigned int ctr=0;
				sensors_config(SENSOR_CHARGE);
				while (sensors_readCapADC() < 1500) { ctr++; }
				
				printf("Time to get 1500: %u\n", ctr);
				
				_delay_ms(500);
				printf("Final CapADC: %u\n", sensors_readCapADC());
				sensors_config(SENSOR_MEASURE);
				printf("Done\n");
				break;
			}
				
			case 'D':
				printf("Discharing cap...\n");
				sensors_config(SENSOR_DISCHARGE);
				_delay_ms(1000);
				sensors_config(SENSOR_MEASURE);
				printf("Done\n");
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

			case 'L': {
				LineFollowResults results;
				
				linesensor_read(linebuf);
				debug_resetTimer();
				linefollow_computeResults(linebuf, results);
				uint16_t computeResults_time = debug_getTimer();
				
				printf("light: ");
				for (int i=0; i<linesensor_count; i++)
					printf("%f ", (double)results.light[i]);
				printf("\n");
				printf("max: %f\n", (double)results.light_max);
				
				printf("squaresum: %e\n", (double)results.squaresum);
				printf("squaretotal: %e\n", (double)results.squaretotal);
				printf("steer: %f\n", (double)results.steer);
				
				static const char *featurestrs[] = {
					"NONE",
					"INTERSECTION",
					"LEFTTURN",
					"RIGHTTURN",
					"NOLINE"
				};
				printf("feature: %s\n", featurestrs[results.feature]);
				printf("Compute Results Time: %u\n", computeResults_time);
				break;
			}
			
			case 't': {
				uint16_t reg;
				if (temp_get16(0x07, reg))
					printf("temperature reg: %u\n", reg);
				else
					printf("failed to get register\n");
				break;
			}
			
			case 'q':
				return;
		}
	}
}

void controlpanel_algorithm() {
	while (true) {
		printf("Algorithm > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'i':
				linefollow_intersection();
				drive_stop();
				break;
				
			case 's': {
				char linebuf[10];
				int steer;
				printf("Steer (%%): ");
				gets(linebuf);
				sscanf(linebuf, "%d", &steer);
				printf("%d\n", steer);
				drive_steer(steer/100.0f, 20);
				getchar();
				drive_stop();
				break;
			}
			
			case 'q':
				return;
		}
	}
}	

void controlpanel_tests() {
	while (true) {
		printf("Tests > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'P':
				controlpanel_pwmtest();
				break;
				
			case 'q':
				return;
		}
	}
}

void controlpanel_pwmtest() {
	for (int16_t pwm = 0; pwm < motor_maxpwm; pwm++)
		controlpanel_pwmtest_single(pwm);
		
	motor_setpwm(MOTOR_LEFT, 0);
	motor_setpwm(MOTOR_RIGHT, 0);
	_delay_ms(2000);
	
	for (int16_t pwm = 0; pwm > -motor_maxpwm; pwm--)
		controlpanel_pwmtest_single(pwm);
		
	motor_setpwm(MOTOR_LEFT, 0);
	motor_setpwm(MOTOR_RIGHT, 0);
}

void controlpanel_pwmtest_single(int16_t pwm) {
	enc_reset(MOTOR_LEFT);
	enc_reset(MOTOR_RIGHT);
	motor_setpwm(MOTOR_LEFT, pwm);
	motor_setpwm(MOTOR_RIGHT, pwm);
	_delay_ms(10);
	printf("%d %d %d\n", pwm, enc_get(MOTOR_LEFT), enc_get(MOTOR_RIGHT));
}

