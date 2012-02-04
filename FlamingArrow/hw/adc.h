#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void adc_init();

uint16_t adc_sample(uint8_t pin);

float adc_sample_float(uint8_t pin);

#endif
