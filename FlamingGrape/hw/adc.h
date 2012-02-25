#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

enum {
	ADC_FRONT_LEFT_RANGE=4,
	ADC_FRONT_RIGHT_RANGE=6,
	ADC_SIDE_LEFT_RANGE=3,		// Not on yet
	ADC_SIDE_RIGHT_RANGE=5,
	ADC_BATTERY=7
};

void adc_init();

uint16_t adc_sample(uint8_t pin);
uint16_t adc_sample_average(uint8_t pin, uint8_t samples);
float adc_sampleFloat(uint8_t pin);

float adc_getBattery();

float adc_sampleRangeFinder(uint8_t pin);

#endif
