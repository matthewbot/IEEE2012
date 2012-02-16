#include "util.h"
#include <stdio.h>
#include <util/delay.h>

void msleep(unsigned long ms) {
	while (ms >= 10) {
		_delay_ms(10);
		ms -= 10;
	}
	
	while (ms-- > 0)
		_delay_ms(1);
}

int vsscanf(const char *s, const char *fmt, va_list ap) {
	FILE f;
	f.flags = __SRD | __SSTR;
	f.buf = (char *)s;
	return vfscanf(&f, fmt, ap);
}
