#ifndef UTIL_H_
#define UTIL_H_

#include <avr/io.h>

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

#endif /* UTIL_H_ */
