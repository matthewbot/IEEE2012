#ifndef LINESENSOR_H_
#define LINESENSOR_H_

#include <stdint.h>

void linesensor_init();

static const int linesensor_count = 8;

void linesensor_setEnabled(bool enabled=true);
uint16_t linesensor_get(int sensor);
void line_cal();
void line_follow();
float get_line_pos();

#endif /* LINESENSOR_H_ */