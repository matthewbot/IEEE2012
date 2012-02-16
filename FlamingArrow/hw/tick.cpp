#include "tick.h"
#include <avr/io.h>
#include <avr/interrupt.h>

static TC1_t &tim = TCF1;
#define TIMOVFVEC TCF1_OVF_vect
#define TIMCCAVEC TCF1_CCA_vect

static volatile uint32_t tickcount;

void tick_init() {
	tim.CTRLA = TC_CLKSEL_DIV8_gc; // 32Mhz / 8 = 4 Mhz timer (TICK_TIMHZ == 4E6)
	tim.CTRLB = TC0_CCAEN_bm; // enable capture compare A
	tim.INTCTRLA = TC_OVFINTLVL_LO_gc; // overflow interrupt enabled at low priority, for running the ticks
	tim.INTCTRLB = TC_CCAINTLVL_HI_gc; // capture compare A interrupt enabled at high priority, because the tick may not be completed before this goes off
	tim.PER = TICK_TIMMAX; // TICK_TIMHZ / (TICK_TIMHZ / TICK_HZ) = TICK_HZ timer
	tim.CCABUF = 200; // 200 / 4Mhz = 50us CCA (for linesensor)
}

void tick_wait() {
	uint32_t t = tickcount;
	while (t == tickcount) { }
}

void tick_halt() {
	tim.CTRLA &= ~TC0_CLKSEL_gm;
}

uint16_t tick_getTimer() {
	return tim.CNT;
}

uint32_t tick_getCount() {
	return tickcount;
}

#include "control/motorcontrol.h"
#include "control/linefollow.h"
#include "debug/debug.h"
#include "hw/linesensor.h"
#include "debug/debug.h"

ISR(TIMOVFVEC) {
	debug_setLED(true);
	tickcount++;
	
	linesensor_tick();
	linefollow_tick();
	motorcontrol_tick();
	debug_tick();
	debug_setLED(false);
}

ISR(TIMCCAVEC) {
	linesensor_tick50us();
}

