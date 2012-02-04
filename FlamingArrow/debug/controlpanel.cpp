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
#include "hw/sensors.h"

#include "debug/controlpanel.h"

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
			case 'q':
				printf("Quitting...\n");
				return;
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
				
			case 'i':
				motor_setpwm(MOTOR_RIGHT, motor_maxpwm/2);
				break;
			case 'k':
				motor_setpwm(MOTOR_RIGHT, 0);
				break;
			case 'm':
				motor_setpwm(MOTOR_RIGHT, -motor_maxpwm/2);
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
				printf("CapADC: %f\n", (double)sensors_readCapADC());
				break;

			case 'v':
				printf("VoltageADC: %f\n", (double)sensors_readVoltageADC());
				break;

			case 's':
				printf("SignalADC: %f\n", (double)sensors_readSignalADC());
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
				linefollow_computeResults(linebuf, results);
				
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
//*/
