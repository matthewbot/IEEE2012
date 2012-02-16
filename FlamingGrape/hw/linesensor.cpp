#include "hw/linesensor.h"
#include "hw/tick.h"
#include "util.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

// configuration

static PORT_t &sigport = PORTD;
#define SIGINT0VEC PORTD_INT0_vect
static const int sigpins_mask = 0xFF; // all 8 pins are signal pins

static PORT_t &ctrlport = PORTC;
static const int ctrlpin = 7;

// variables

static volatile uint8_t prevmask;
static volatile uint8_t readingctr;
static volatile uint16_t readings[8];
static volatile uint16_t readingsbuf[8];

void linesensor_init() {
	ctrlport.OUTSET = _BV(ctrlpin); // control pin high to turn on array
	ctrlport.DIRSET = _BV(ctrlpin);

	sigport.OUTSET = sigpins_mask; // all pins are high when configured as outputs
	PORTCFG.MPCMASK = sigpins_mask; // use multi-pin configuation to configure all signal pins the same way
	sigport.PIN0CTRL = PORT_ISC_FALLING_gc; // configure all pins for falling edge sensing
	sigport.INT0MASK = sigpins_mask; // configure all signal pins to be a part of the port's interrupt zero
}

void linesensor_setLEDs(bool enabled) {
	if (enabled)
		ctrlport.OUTSET = _BV(ctrlpin);
	else
		ctrlport.OUTCLR = _BV(ctrlpin);
}

void linesensor_read(uint16_t *buf) {
	tick_suspend();
	for (int i=0; i<8; i++) {
		buf[i] = readingsbuf[7-i];
	}
	tick_resume();
}

#pragma GCC optimize("3") // jack up optimization for these ISRs, in particular the signal port ISR

void linesensor_tick() {
	sigport.INTCTRL &= ~PORT_INT0LVL_gm; // disable the pin change interrupt
	for (int i=0; i<8; i++) { // for each pin
		if (prevmask & (uint8_t)_BV(i)) // if it never got a reading
			readingsbuf[i] = UINT16_MAX; // give it maximum value
		else
			readingsbuf[i] = readings[i]; // otherwise give it a reading
	}
	
	prevmask = sigpins_mask;

	sigport.DIRSET = sigpins_mask; // begin charging by setting pins as outputs
	readingctr++;
}

void linesensor_tick50us() {
	sigport.DIRCLR = sigpins_mask; // begin discharging by setting pins as inputs
	sigport.INTCTRL |= PORT_INT0LVL_HI_gc; // enable the pin change interrupt
}

ISR(SIGINT0VEC) {
	uint16_t timval = tick_getTimer();
	sigport.INTFLAGS = PORT_INT0IF_bm; // clear interrupt flag

	uint8_t curmask = sigport.IN & sigpins_mask; // find changed pins by comparing the current mask with the previous mask
	uint8_t changed = ~curmask & prevmask;
	prevmask = curmask;

	for (int i=0; i<8; i++) { // record the timer value for each changed pin
		if (changed & (uint8_t)_BV(i)) // casting to uint8_t allows gcc to use bit testing instructions
			readings[i] = timval;
	}
}

