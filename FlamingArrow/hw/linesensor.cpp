#include <stdlib.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "debug/debug.h"
#include "control/linefollow.h"
#include "debug/controlpanel.h"

#include "hw/linesensor.h"
#include "util.h"

// configuration

static PORT_t &sigport = PORTD;
#define SIGINT0VEC PORTD_INT0_vect
static const int sigpins_mask = 0xFF; // all 8 pins are signal pins

static PORT_t &ctrlport = PORTC;
static const int ctrlpin = 7;

static TC1_t &tim = TCD1;
#define TIMOVFVEC TCD1_OVF_vect
#define TIMCCAVEC TCD1_CCA_vect

// variables

static volatile uint8_t prevmask;
static volatile uint8_t readingctr;
static volatile uint16_t readings[8];
static volatile uint16_t readingsbuf[8];

// utility functions

static void linesensor_wait();

void linesensor_init() {
	ctrlport.OUTSET = _BV(ctrlpin); // control pin high to turn on array
	ctrlport.DIRSET = _BV(ctrlpin);

	sigport.OUTSET = sigpins_mask; // all pins are high when configured as outputs
	PORTCFG.MPCMASK = sigpins_mask; // use multi-pin configuation to configure all signal pins the same way
	sigport.PIN0CTRL = PORT_ISC_FALLING_gc; // configure all pins for falling edge sensing
	sigport.INT0MASK = sigpins_mask; // configure all signal pins to be a part of the port's interrupt zero
	sigport.INTCTRL = PORT_INT0LVL_HI_gc; // configure interrupt zero to a high priority interrupt

	tim.CTRLA = TC_CLKSEL_DIV8_gc; // 32Mhz / 8 = 4 Mhz timer
	tim.CTRLB = TC0_CCAEN_bm; // enable capture compare A
	tim.INTCTRLA = TC_OVFINTLVL_MED_gc; // overflow interrupt enabled at high priority, used to start the charging period
	tim.INTCTRLB = TC_CCAINTLVL_HI_gc; // capture compare A interrupt enabled at high priority, used to end the charging period
	tim.PER = 40000; // 4Mhz / 40000 = 100hz overflow, 10ms period
	tim.CCABUF = 2000; // 2000 / 4Mhz = 500us charge period
}

void linesensor_setLEDs(bool enabled) {
	if (enabled)
		ctrlport.OUTSET = _BV(ctrlpin);
	else
		ctrlport.OUTCLR = _BV(ctrlpin);
}

void linesensor_read(uint16_t *buf) {
	linesensor_wait();
	
	for (int i=0; i<8; i++) {
		buf[i] = readingsbuf[7-i];
	}
}

static void linesensor_wait() {
	uint8_t ctr = readingctr;
	while (readingctr == ctr) { } // wait for reading counter to change
}

#pragma GCC optimize("3") // jack up optimization for these ISRs, in particular the signal port ISR

ISR(TIMOVFVEC) {
	for (int i=0; i<8; i++) { // for each pin
		if (prevmask & (uint8_t)_BV(i)) // if it never got a reading
			readingsbuf[i] = linesensor_maxval; // give it maxval
		else
			readingsbuf[i] = readings[i]; // otherwise give it a reading
	}
	prevmask = sigpins_mask;

	sigport.DIRSET = sigpins_mask; // begin charging by setting pins as outputs
	readingctr++;
}

ISR(TIMCCAVEC) {
	sigport.DIRCLR = sigpins_mask; // begin discharging by setting pins as inputs
}

ISR(SIGINT0VEC) {
	debug_setLED(true);
	uint16_t timval = tim.CNT;
	sigport.INTFLAGS = PORT_INT0IF_bm; // clear interrupt flag

	uint8_t curmask = sigport.IN & sigpins_mask; // find changed pins by comparing the current mask with the previous mask
	uint8_t changed = ~curmask & prevmask;
	prevmask = curmask;

	for (int i=0; i<8; i++) { // record the timer value for each changed pin
		if (changed & (uint8_t)_BV(i)) // casting to uint8_t allows gcc to use bit testing instructions
			readings[i] = timval;
	}
}

