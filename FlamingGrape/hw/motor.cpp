#include "hw/motor.h"
#include <avr/io.h>
#include <stdio.h>

static PORT_t &ctrlport = PORTK;
static const int ctrlpins_mask = 0xFF;

static PORT_t &pwmport = PORTF;
static const int pwmpins_mask = 0x0F;
static TC0_t &pwmtim = TCF0;

static const bool flip[4] = {false, false, false, true};

void motor_init() {
	ctrlport.DIRSET = ctrlpins_mask;
	pwmport.DIRSET = pwmpins_mask;

	pwmtim.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	pwmtim.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	pwmtim.PER = 1023; // 32Mhz / ~1024 = 31.25 khz pwm freq
}

void motor_setpwm(uint8_t mot, int16_t pwm) {
	uint8_t in1pin_mask = _BV(2*mot);
	uint8_t in2pin_mask = in1pin_mask << 1;
	register16_t &ccreg = (&pwmtim.CCABUF)[mot]; // CCxBUF registers are adjacent in memory

	if (flip[mot])
		pwm = -pwm;

	if (pwm == 0) {
		ctrlport.OUTCLR = in1pin_mask | in2pin_mask;
		ccreg = 0;
	} else if (pwm > 0) {
		ctrlport.OUTCLR = in2pin_mask;
		ctrlport.OUTSET = in1pin_mask;
		ccreg = pwm;
	} else {
		ctrlport.OUTCLR = in1pin_mask;
		ctrlport.OUTSET = in2pin_mask;
		ccreg = -pwm;
	}
}

void motor_allOff() {
	for (int i=0; i<motor_count; i++) {
		(&pwmtim.CCABUF)[i] = 0;
	}
}

int16_t motor_getpwm(uint8_t mot) {
	int16_t pwm = (&pwmtim.CCA)[mot]; // CCx registers are also adjacent

	uint8_t in1pin_mask = _BV(2*mot);
	if (!(ctrlport.IN & in1pin_mask))
		pwm = -pwm;

	if (flip[mot])
		pwm = -pwm;

	return pwm;
}

