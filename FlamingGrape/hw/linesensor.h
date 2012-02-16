#ifndef LINESENSOR_H_
#define LINESENSOR_H_

#include <stdint.h>

static const int linesensor_count = 8;

void linesensor_init();
void linesensor_setLEDs(bool enabled=true);
void linesensor_read(uint16_t *buf);

void linesensor_tick();
void linesensor_tick50us();

#endif /* LINESENSOR_H_ */
