#ifndef UTIL_H_
#define UTIL_H_

#include <avr/io.h>
#include <math.h>
#include <stdarg.h>

inline float sign(float in) {
	if (in > 0) {
		return 1.0;
	} else if (in < 0) {
		return -1.0;
	} else {
		return 0;
	}
}

inline float sqrf(float f) { return f*f; }

inline float degtorad(float deg) { return deg * M_PI / 180; }
inline float radtodeg(float rad) { return rad * 180 / M_PI; }

float anglewrap(float rad);

void msleep(unsigned long ms);

int vsscanf(const char *s, const char *fmt, va_list ap); // avr-libc is missing this for some reason

#endif /* UTIL_H_ */
