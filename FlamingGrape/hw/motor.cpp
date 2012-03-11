#include "hw/motor.h"
#include <avr/io.h>
#include <stdio.h>

static PORT_t &ctrlport = PORTK;
static const int ctrlpins_mask = 0xFF;

static PORT_t &PWMport = PORTF;
static const int PWMpins_mask = 0x0F;
static TC0_t &PWMtim = TCF0;

static const bool flip[4] = {true, false, false, false}; // LRDF
static const uint8_t port[4] = {2, 3, 1, 0}; // LRDF

void motor_init() {
	ctrlport.DIRSET = ctrlpins_mask;
	PWMport.DIRSET = PWMpins_mask;

	PWMtim.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	PWMtim.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	PWMtim.PER = 1023; // 32Mhz / ~1024 = 31.25 khz PWM freq
	
	motor_setPWM(MOTOR_FAN, motor_maxPWM);
}

void motor_setPWM(uint8_t mot, int16_t PWM) {
	uint8_t in1pin_mask = _BV(2*port[mot]);
	uint8_t in2pin_mask = in1pin_mask << 1;
	register16_t &ccreg = (&PWMtim.CCABUF)[port[mot]]; // CCxBUF registers are adjacent in memory

	if (flip[mot])
		PWM = -PWM;

	if (PWM == 0) {
		ctrlport.OUTCLR = in1pin_mask | in2pin_mask;
		ccreg = 0;
	} else if (PWM > 0) {
		ctrlport.OUTCLR = in2pin_mask;
		ctrlport.OUTSET = in1pin_mask;
		ccreg = PWM;
	} else {
		ctrlport.OUTCLR = in1pin_mask;
		ctrlport.OUTSET = in2pin_mask;
		ccreg = -PWM;
	}
}

void motor_allOff() {
	for (int i=0; i<motor_count; i++) {
		(&PWMtim.CCABUF)[i] = 0;
	}
}

int16_t motor_getPWM(uint8_t mot) {
	int16_t PWM = (&PWMtim.CCA)[port[mot]]; // CCx registers are also adjacent

	uint8_t in1pin_mask = _BV(2*port[mot]);
	if (!(ctrlport.IN & in1pin_mask))
		PWM = -PWM;

	if (flip[mot])
		PWM = -PWM;

	return PWM;
}
