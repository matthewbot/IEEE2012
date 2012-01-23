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
				
				printf("raw: ");
				for (int i=0; i<linesensor_count; i++)
					printf("%f ", (double)results.raw_light[i]);
				printf("\nraw min: %f\n", (double)results.raw_min);
				
				printf("light: ");
				for (int i=0; i<linesensor_count; i++)
					printf("%f ", (double)results.light[i]);
				printf("\n");
				
				printf("squaresum: %e\n", (double)results.squaresum);
				printf("squaretotal: %e\n", (double)results.squaretotal);
				printf("max: %f\n", (double)results.max);
				printf("steer: %f\n", (double)results.steer);
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
			case 'b':
				linefollow_bump();
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
