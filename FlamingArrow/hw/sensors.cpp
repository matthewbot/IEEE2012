#include "hw/sensors.h"
#include "hw/adc.h"
#include <avr/io.h>

static PORT_t &ctrlport = PORTC;
static const int ctrlpins_outmask = 0x70;

static const int ctrlpins_capcharge =    _BV(4);
static const int ctrlpins_capground =    _BV(5);
static const int ctrlpins_capdischarge = _BV(6);
static const int ctrlpins_frontbump =    _BV(0);

static const int adcpin_cap = 0;
static const int adcpin_voltage = 1;
static const int adcpin_signal = 2;

void sensors_init() {
	PORTCFG.MPCMASK = ctrlpins_frontbump;
	ctrlport.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm; // enable pull up resistors and inversion on switch
	
	PORTCFG.MPCMASK = ctrlpins_capcharge;
	ctrlport.PIN0CTRL = PORT_INVEN_bm; // enable inversion on cap charge

	ctrlport.DIRSET = ctrlpins_outmask; // set output pins
}

bool sensors_readBump() {
	return (ctrlport.IN & ctrlpins_frontbump) != 0;
}

uint16_t sensors_readCapADC() {
	return adc_sample(adcpin_cap);
}
uint16_t sensors_readVoltageADC() {
	return adc_sample(adcpin_voltage);
}
uint16_t sensors_readSignalADC() {
	return adc_sample(adcpin_signal);
}

void sensors_config(SensorConfig config) {
	static const uint8_t masks[] = {
		0, // SENSOR_MEASURE
		ctrlpins_capground | ctrlpins_capcharge, // SENSOR_CHARGE
		ctrlpins_capground | ctrlpins_capdischarge, // SENSOR_DISCHARGE
	};
	
	uint8_t out = ctrlport.OUT;
	out &= ~ctrlpins_outmask;
	out |= masks[config];
	ctrlport.OUT = out;
}
