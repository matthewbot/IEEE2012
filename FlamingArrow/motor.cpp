#include "motor.h"
#include <avr/io.h>

static PORT_t &ctrlport = PORTK;
static const int ctrlpins_mask = 0xFF;

static PORT_t &pwmport = PORTF;
static const int pwmpins_mask = 0x0F;
static TC0_t &pwmtim = TCF0;

void motor_init() {
	ctrlport.DIRSET = ctrlpins_mask;
	pwmport.DIRSET = pwmpins_mask;
	
	pwmtim.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	pwmtim.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	pwmtim.PER = 1024; // 32Mhz / 1024 = 31.25 khz pwm freq
}

void motor_setpwm(uint8_t mot, int16_t pwm) {
	uint8_t in1pin_mask = _BV(mot);
	uint8_t in2pin_mask = in1pin_mask << 1;
	register16_t &ccreg = (&pwmtim.CCABUF)[mot]; // CCxBUF registers are adjacent in memory
	
	if (pwm == 0) {
		pwmport.OUTCLR = in1pin_mask | in2pin_mask;
		ccreg = 0;
	} else if (pwm > 0) {
		pwmport.OUTCLR = in2pin_mask;
		pwmport.OUTSET = in1pin_mask;
		ccreg = pwm;
	} else {
		pwmport.OUTCLR = in1pin_mask;
		pwmport.OUTSET = in2pin_mask;
		ccreg = -pwm;
	}
}
