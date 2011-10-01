#include <avr/io.h>

#include "debug.h"
#include "temp.h"

#include "controlpanel.h"

void controlpanel_init() {
	debug_puts("Starting up\r\n");
}

void controlpanel_motor() {
	char ch = 'a';
	
	while (ch != 'q') {
		debug_puts("Motor >");
		debug_getch();
		switch (ch) {
			default:
				debug_puts("ACTIONS NOT CODED");
				break;
		}
	}
}

void controlpanel_temp() {
	char ch = 'a';
	
	while (ch != 'q') {
		debug_puts("Temperature >");
		debug_getch();
		switch (ch) {
			case '1':
				debug_printf("Ambient Raw: %f\n", (double)temp_getraw(AMB));
				break;
			case '2':
				debug_printf("Object Raw: %f\n", (double)temp_getraw(OBJ));
				break;
			case '3':
				debug_printf("Ambient: %f\n", (double)temp_get(AMB));
				break;
			case '4':
				debug_printf("Object: %f\n", (double)temp_get(OBJ));
				break;
			default:
				debug_puts("ACTIONS NOT CODED");
				break;
		}
	}
}

void controlpanel_linesensor() {
	char ch = 'a';
	
	while (ch != 'q') {
		debug_puts("Line Sensor >");
		debug_getch();
		switch (ch) {
			default:
				debug_puts("ACTIONS NOT CODED");
				break;
		}
	}
}

void controlpanel() {
	char ch = 'a';
	
	while(ch != 'q') {
		debug_puts("Main >");
		debug_getch();
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
	debug_puts("Quitting...");
}
