#ifndef TESTS_H
#define TESTS_H

#include <stdint.h>

void tests_PWM();
void tests_PWM_single(int16_t PWM);
void tests_mag();
void tests_magfollow();
void tests_led();
void tests_linefollow();
void tests_movingLineRead();
void tests_linesensorMin(uint16_t *min);

#endif
