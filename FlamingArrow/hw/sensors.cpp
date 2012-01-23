#include "hw/sensors.h"
#include "hw/adc.h"
#include <avr/io.h>

static PORT_t &ctrlport = PORTC;
static const int ctrlpins_mask = 0x70;

static const int ctrlpins_capcharge = _BV(6);
static const int ctrlpins_capground = _BV(5);
static const int ctrlpins_capdischarge = _BV(4);
static const int ctrlpins_frontbump = _BV(0);

static const int adcpin_cap = 0;
static const int adcpin_voltage = 1;
static const int adcpin_signal = 2;

void sensors_init() {
	ctrlport.DIRSET = ctrlpins_mask;
	ctrlport.OUTSET = ctrlpins_capcharge;
	
	PORTCFG.MPCMASK = ctrlpins_frontbump;
	ctrlport.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm; // enable pull up resistors and inversion on switch
}

bool sensors_readBump() {
	return (ctrlport.IN & ctrlpins_frontbump) != 0;
}

float sensors_readCapADC() {
	return adc_sample(adcpin_cap)/4095.0f * 5;
}
float sensors_readVoltageADC() {
	return adc_sample(adcpin_voltage)/4095.0f * 5;
}
float sensors_readSignalADC() {
	return adc_sample(adcpin_signal)/4095.0f * 5;
}

