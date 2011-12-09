#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <avr/io.h>
#include "hw/enc.h"

static PORT_t &encport = PORTF;
static const int encpins_mask = 0xF0;

static const int chan0mux = EVSYS_CHMUX_PORTF_PIN4_gc;
static const int chan2mux = EVSYS_CHMUX_PORTF_PIN6_gc; // event channels are used in pairs for qdec

static TC0_t &enctim0 = TCE0;
static TC1_t &enctim1 = TCE1;

void enc_init() {
	PORTCFG.MPCMASK = encpins_mask; // configure all encoder pins
	encport.PIN0CTRL = PORT_ISC_LEVEL_gc; // set them to level sensing, required for qdec hardware

	EVSYS.CH0MUX = chan0mux; // configure the event channel muxes to the correct pins
	EVSYS.CH2MUX = chan2mux;
	EVSYS.CH0CTRL = EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_8SAMPLES_gc; // turn on quadrature decoding, and digital filtering

	enctim0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc; // set up timers for quadrature decoding from the correct event channels
	enctim1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;
	enctim0.PER = enctim1.PER = 0xFFFF; // max out the period so we use all 16 bits before overflowing
	enctim0.CTRLA = enctim1.CTRLA = TC_CLKSEL_DIV1_gc; // div1 clock selection required for qdec to work
}

uint16_t enc_get(uint8_t num) {
	if (num == 0)
		return enctim0.CNT;
	else
		return enctim1.CNT;
}

void enc_reset(uint8_t num) {
	if (num == 0)
		enctim0.CNT = 0;
	else
		enctim1.CNT = 0;
}

int16_t enc_diff(uint16_t a, uint16_t b) {
	int16_t diff = (int16_t)a - (int16_t)b;
	if (diff > INT16_MAX/2) {	// if encoder wrapped around
		diff -= INT16_MAX;
	} else if (diff < INT16_MIN/2) {
		diff += INT16_MAX;
	}
	return diff;
}

