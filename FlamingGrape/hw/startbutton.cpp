#include "hw/startbutton.h"
#include "hw/uart.h"
#include <avr/io.h>

static PORT_t &port = PORTC;
static const int mask = _BV(5);
static uint8_t debounce_ctr=0;

void startbutton_init() {
	PORTCFG.MPCMASK = mask;
	port.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

bool startbutton_isPressed() {
	return !(port.IN & mask);
}

void startbutton_tick() {
	if (debounce_ctr) {
		debounce_ctr--;
		return;
	}
	
	if (!(port.IN & mask)) {
		debounce_ctr = 100;
		uart_putch(UART_USB, 'G');
	}
}
