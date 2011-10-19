#include <stdlib.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "debug.h"
#include "linefollow.h"

#include "linesensor.h"

#pragma GCC optimize("3") // jack up optimization for these ISRs, in particular the signal port ISR

void linesensor_gotReadings();

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
static volatile uint16_t readings[8];

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

void linesensor_setEnabled(bool enabled) {
	if (enabled)
		ctrlport.OUTSET = _BV(ctrlpin);
	else
		ctrlport.OUTCLR = _BV(ctrlpin);
}

ISR(TIMOVFVEC) {
	sigport.INTCTRL = PORT_INT0LVL_OFF_gc; // disable edge interrupt
	tim.CTRLA = 0; // stop timer
	
	for (int i=0; i<8; i++)
		if (prevmask & (uint8_t)_BV(i)) // casting to uint8_t allows gcc to use bit testing instructions
			readings[i] = 40000; // pin never changed state, so set maximum value
	
	linesensor_gotReadings();
	
	sigport.DIRSET = sigpins_mask; // begin charging by setting pins as outputs
	prevmask = sigpins_mask;
	for (int i=0; i<8; i++)
		readings[i] = 65535;
	
	tim.CTRLA = TC_CLKSEL_DIV8_gc; // restart timer
}

ISR(TIMCCAVEC) {
	sigport.DIRCLR = sigpins_mask; // begin discharging by setting pins as inputs
	sigport.INTCTRL = PORT_INT0LVL_HI_gc; // enable edge interrupt at high priority
	sigport.INTFLAGS = PORT_INT0IF_bm; // clear interrupt flag
}

ISR(SIGINT0VEC) {
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

void linesensor_gotReadings() {
	linefollow_sensorUpdate((uint16_t*)readings);
	
	/* debug_printf("%u %u %u %u %u %u %u %u\r\n",
		readings[0],
		readings[1],
		readings[2],
		readings[3],
		readings[4],
		readings[5],
		readings[6],
		readings[7]
	); */
}
