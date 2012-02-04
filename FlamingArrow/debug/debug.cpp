#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "util.h"

#include "debug/debug.h"

// debug LED
static PORT_t &ledport = PORTR;
static const int ledpin = 1;

// debug/USB uart
static USART_t &uart_usb = USARTC0;
#define RXVEC_USB USARTC0_RXC_vect
static PORT_t &uartport_usb = PORTC;
static const int txpin_usb = 3;
static const int rxpin_usb = 2;
static const int bsel_usb = 2158; // makes 115200 baud
static const int bscale_usb = 0xA;

static PORT_t &uartport_xbee = PORTE;
#define RXVEC_XBEE USARTE1_RXC_vect
static USART_t &uart_xbee = USARTE1;
static const int bsel_xbee = 3333;
static const int bscale_xbee = 0xC;
static const int txpin_xbee = 7;
static const int rxpin_xbee = 6;

// debug timer
static TC1_t &tim = TCC1;

static char recvbuf[8];
static volatile uint8_t recvbuf_pos;

static int myput(char ch, FILE* file) {
	if(ch == '\n')
		myput('\r', file);
	while (!(uart_usb.STATUS & USART_DREIF_bm)) { }
	uart_usb.DATA = ch;
	while (!(uart_xbee.STATUS & USART_DREIF_bm)) { }
	uart_xbee.DATA = ch;
	return 0;
}

static int myget(FILE* file) {
	while (recvbuf_pos == 0) { }
	
	util_cli_lo();
	char ch = recvbuf[0];
	memmove(recvbuf, recvbuf+1, recvbuf_pos - 1);
	recvbuf_pos--;
	util_sei_lo();
	
	if (ch == '\r')
		ch = '\n';
	
	return ch;
}

void debug_init() {
	ledport.DIRSET = _BV(ledpin);
	debug_setLED(false);
	
	uartport_usb.OUTSET = _BV(txpin_usb); // make pin high to avoid transmitting a false start bit on startup
	uartport_usb.DIRSET = _BV(txpin_usb);
	
	uart_usb.CTRLA = USART_RXCINTLVL_LO_gc;
	uart_usb.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	uart_usb.CTRLC = USART_CHSIZE_8BIT_gc;
	uart_usb.BAUDCTRLA = bsel_usb & 0xFF;
	uart_usb.BAUDCTRLB = (bscale_usb << USART_BSCALE_gp) | (bsel_usb >> 8);
	
	uartport_xbee.OUTSET = _BV(txpin_xbee);
	uartport_xbee.DIRSET = _BV(txpin_xbee);
	
	uart_xbee.CTRLA = USART_RXCINTLVL_LO_gc;
	uart_xbee.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	uart_xbee.CTRLC = USART_CHSIZE_8BIT_gc;
	uart_xbee.BAUDCTRLA = bsel_xbee & 0xFF;
	uart_xbee.BAUDCTRLB = (bscale_xbee << USART_BSCALE_gp) | (bsel_xbee >> 8);
	
	tim.CTRLA = TC_CLKSEL_DIV64_gc; // 32Mhz / 32 = 1 Mhz timer
	tim.PER = 0xFFFF; // 1Mhz / 65536
	
	fdevopen(myput, myget);
}

void debug_setLED(bool on) {
	if (on)
		ledport.OUTCLR = _BV(ledpin);
	else
		ledport.OUTSET = _BV(ledpin);
}

void debug_resetTimer() {
	tim.CNT = 0;
}

uint16_t debug_getTimer() {
	return tim.CNT / 1000;
}

static void receive(uint8_t ch) {
	if (recvbuf_pos >= sizeof(recvbuf))
		return;
	
	recvbuf[recvbuf_pos++] = ch;
}

ISR(RXVEC_USB) {
	receive(uart_usb.DATA);
}

ISR(RXVEC_XBEE) {
	debug_setLED(true);
	receive(uart_xbee.DATA);
}
