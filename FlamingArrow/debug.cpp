#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "util.h"

#include "debug.h"

// debug LED
static PORT_t &ledport = PORTR;
static const int ledpin = 1;

// debug/USB uart
static USART_t &uart = USARTC0;
#define RXVEC USARTC0_RXC_vect
static PORT_t &uartport = PORTC;
static const int txpin = 3;
static const int rxpin = 2;
static const int bsel = 2158; // makes 115200 baud
static const int bscale = 0xA;

static char recvbuf[8];
static volatile uint8_t recvbuf_pos;

static int myput(char ch, FILE* file) {
	if(ch == '\n')
		myput('\r', file);
	while (!(uart.STATUS & USART_DREIF_bm)) { }
	uart.DATA = ch;
	return 0;
}

static int myget(FILE* file) {
	while (recvbuf_pos == 0) { }
	
	util_cli_lo();
	char ch = recvbuf[0];
	memmove(recvbuf, recvbuf+1, recvbuf_pos - 1);
	recvbuf_pos--;
	util_sei_lo();
	
	return ch;
}

void debug_init() {
	ledport.DIRSET = _BV(ledpin);
	debug_setLED(false);
	
	uartport.OUTSET = _BV(txpin); // make pin high to avoid transmitting a false start bit on startup
	uartport.DIRSET = _BV(txpin);
	uart.CTRLA = USART_RXCINTLVL_LO_gc;
	uart.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	uart.CTRLC = USART_CHSIZE_8BIT_gc;
	uart.BAUDCTRLA = bsel & 0xFF;
	uart.BAUDCTRLB = (bscale << USART_BSCALE_gp) | (bsel >> 8);
	
	fdevopen(myput, myget);
}

void debug_setLED(bool on) {
	if (on)
		ledport.OUTCLR = _BV(ledpin);
	else
		ledport.OUTSET = _BV(ledpin);
}

ISR(RXVEC) {
	uint8_t ch = uart.DATA;
	if (recvbuf_pos >= sizeof(recvbuf))
		return;
	
	recvbuf[recvbuf_pos++] = ch;
}
