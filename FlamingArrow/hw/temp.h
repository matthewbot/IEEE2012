#ifndef TEMP_H_
#define TEMP_H_

#include <stdint.h>

void temp_test();

bool temp_getROM16(uint8_t addr, uint16_t &val);
bool temp_get16(uint8_t addr, uint16_t &val);

#endif /* TEMP_H_ */
