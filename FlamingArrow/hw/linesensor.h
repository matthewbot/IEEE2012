#ifndef LINESENSOR_H_
#define LINESENSOR_H_

static const int linesensor_count = 8;
static const uint16_t linesensor_maxval = 40000;

void linesensor_init();
void linesensor_setLEDs(bool enabled=true);
void linesensor_read(uint16_t *buf);

#endif /* LINESENSOR_H_ */
