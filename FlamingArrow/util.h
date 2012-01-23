#ifndef UTIL_H_
#define UTIL_H_

#include <avr/io.h>
#include <math.h>

// low priority interrupt control, leaves medium and high interrupts active
inline void util_cli_lo() { PMIC.CTRL &= ~PMIC_LOLVLEN_bm; }
inline void util_sei_lo() { PMIC.CTRL |= PMIC_LOLVLEN_bm; }

inline float sign(float in) {
	if (in > 0) {
		return 1.0;
	} else if (in < 0) {
		return -1.0;
	} else {
		return 0;
	}
}

inline int min(int a, int b) { return (a < b) ? a : b; }

inline float degtorad(float deg) { return deg * M_PI / 180; }
inline float radtodeg(float rad) { return rad * 180 / M_PI; }

void msleep(unsigned long ms);

#endif /* UTIL_H_ */
