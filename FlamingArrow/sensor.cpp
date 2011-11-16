#include "sensor.h"
#include "adc.h"
#include <avr/io.h>

static PORT_t &ctrlport = PORTC;
static const int ctrlpins_mask = 0x70;

static const int ctrlpins_capcharge = _BV(6);
static const int ctrlpins_capground = _BV(5);
static const int ctrlpins_capdischarge = _BV(4);

static const int adcpin_cap = 0;
static const int adcpin_voltage = 1;
static const int adcpin_signal = 2;

void sensor_init() {
	ctrlport.DIRSET = ctrlpins_mask;
	ctrlport.OUTSET = ctrlpins_capcharge;
}

float sensor_readCapADC() {
	return adc_sample(adcpin_cap)/4095.0f * 5;
}
float sensor_readVoltageADC() {
	return adc_sample(adcpin_voltage)/4095.0f * 5;
}
float sensor_readSignalADC() {
	return adc_sample(adcpin_signal)/4095.0f * 5;
}

