#include "init.h"
#include <avr/interrupt.h>
#include <avr/io.h>

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

#include "debug/controlpanel.h"
#include "debug/debug.h"
#include "control/motorcontrol.h"
#include "control/drive.h"
#include "hw/motor.h"
#include "hw/mag.h"
#include "hw/enc.h"
#include "hw/adc.h"
#include "hw/linesensor.h"
#include "hw/tick.h"
#include "hw/uart.h"

void init_modules() {
	uart_init();
	debug_init();
	motor_init();
	enc_init();
	adc_init();
	tick_init();
	linesensor_init();
	mag_init();
	motorcontrol_init();
	controlpanel_init();
}

void init() {
	init_clocks();
	init_modules();
	init_interrupts();
}
