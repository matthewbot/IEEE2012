#include <stdio.h>

#include <avr/io.h>

#include "debug.h"
#include "temp.h"

#include "controlpanel.h"

void controlpanel_init() {
	puts("Starting up\n");
}

void controlpanel_motor() {
	char ch = 'a';
	
	while (ch != 'q') {
		puts("Motor >");
		getchar();
		switch (ch) {
			default:
				puts("ACTIONS NOT CODED");
				break;
		}
	}
}

void controlpanel_temp() {
	char ch = 'a';
	
	while (ch != 'q') {
		puts("Temperature >");
		getchar();
		switch (ch) {
			case '1':
				printf("Ambient Raw: %f\n", (double)temp_getraw(AMB));
				break;
			case '2':
				printf("Object Raw: %f\n", (double)temp_getraw(OBJ));
				break;
			case '3':
				printf("Ambient: %f\n", (double)temp_get(AMB));
				break;
			case '4':
				printf("Object: %f\n", (double)temp_get(OBJ));
				break;
			default:
				puts("ACTIONS NOT CODED");
				break;
		}
	}
}

void controlpanel_linesensor() {
	char ch = 'a';
	
	while (ch != 'q') {
		puts("Line Sensor >");
		getchar();
		switch (ch) {
			default:
				puts("ACTIONS NOT CODED");
				break;
		}
	}
}

void controlpanel() {
	char ch = 'a';
	
	while(ch != 'q') {
		puts("Main >");
		getchar();
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
				ch = 'a';
				break;
			default:
				break;
		}
	}
	puts("Quitting...");
}
