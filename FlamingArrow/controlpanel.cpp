#include <avr/io.h>
#include "controlpanel.h"
#include "debug.h"
#include "temp.h"

void controlpanel_init() {
	debug_puts("Starting up\r\n");	
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
				break;
			default:
				break;	
		}			
	}		
	debug_puts("Quitting...");
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
			case 'o':
				debug_puts("%f\n", temp_getraw(OBJ)*0.20-273.15);
				break;
			case 'a':
				debug_puts("%f\n", temp_getraw(AMB)*0.20-273.15);
				break;
			case 'd':
				debug_puts("f\n", temp_get());
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