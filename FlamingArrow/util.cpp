#include "util.h"

#include <util/delay.h>

void msleep(unsigned long ms) {
	while (ms >= 10) {
		_delay_ms(10);
		ms -= 10;
	}
	
	while (ms-- > 0)
		_delay_ms(1);
}
