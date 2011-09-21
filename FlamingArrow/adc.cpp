#include "adc.h"
#include <avr/io.h>

void adc_init() {
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.REFCTRL = ADC_REFSEL_VCC_gc | ADC_BANDGAP_bm | ADC_TEMPREF_bm;
	ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc; // Tim says the op-amps on port A limit bandwidth to 1 Mhz

	ADCB.CTRLA = ADC_ENABLE_bm;
	ADCB.REFCTRL = ADC_REFSEL_VCC_gc | ADC_BANDGAP_bm | ADC_TEMPREF_bm;
	ADCB.PRESCALER = ADC_PRESCALER_DIV16_gc;
}

uint16_t adc_sample(uint8_t pin) {
	ADC_t *adc;
	if (pin < 8) { // 0-7 use ADCA, 8-15 use ADCB
		adc = &ADCA;
	} else {
		adc = &ADCB;
		pin -= 8;
	}

	adc->CH0.MUXCTRL = pin << ADC_CH_MUXPOS_gp; // select the desired pin
	adc->CH0.CTRL = ADC_CH_START_bm | ADC_CH_INPUTMODE_SINGLEENDED_gc; // start a single ended conversion
	while (!(adc->CH0.INTFLAGS & ADC_CH_CHIF_bm)) { } // wait for it to complete
	adc->CH0.INTFLAGS = ADC_CH_CHIF_bm; // clear completion flag
	return adc->CH0.RES;
}
