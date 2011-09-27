#include "init.h"
#include <avr/io.h>
#include <avr/interrupt.h>

void init() {
	init_clocks();
	init_modules();
	init_interrupts();
}

void init_clocks() {
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm; // enable 32 Mhz clock (while leaving the current 2 Mhz clock enabled)
	while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // wait for it to stabilize
	CPU_CCP = CCP_IOREG_gc; // enable access to protected registers
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to the 32 Mhz clock
}

void init_interrupts() {
	PMIC.CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

#include "debug.h"
#include "motor.h"
#include "enc.h"
#include "adc.h"
#include "linesensor.h"

void init_modules() {
	debug_init();
	motor_init();
	enc_init();
	adc_init();
	linesensor_init();
	
	debug_puts("Starting up!\r\n");
}
