#ifndef LINESENSOR_H_
#define LINESENSOR_H_

#include <stdint.h>

void linesensor_init();

static const int linesensor_count = 8;

void linesensor_setEnabled(bool enabled=true);

#endif /* LINESENSOR_H_ */
