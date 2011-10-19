#include <stdio.h>

#include <avr/io.h>

#include "debug.h"
#include "temp.h"
#include "linefollow.h"

#include "controlpanel.h"

void controlpanel_init() {
	printf("Starting up\n");
}

void controlpanel_motor() {
	while (true) {
		printf("Motor > ");
		char ch = getchar();
		printf("%c\n", ch);
		switch (ch) {
			case 'b':
				return;
			default:
				printf("Unknown. Commands: Back.\n");
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
			case 'b':
				return;
			default:
				printf("Unknown. Commands: Follow, Stop following, Back.\n");
				break;
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
			case 'q':
				printf("Quitting...\n");
				return;
			default:
				printf("Unknown. Submenus: Motor, Temp, Line sensor. Commands: Quit.\n");
				break;
		}
	}
}
