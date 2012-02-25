#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

enum Motor {
	MOTOR_LEFT,
	MOTOR_RIGHT,
	MOTOR_DEPLOY = 3
};

static const int motor_count = 4;
static const int16_t motor_maxpwm = 1024;

void motor_init();
void motor_setpwm(uint8_t mot, int16_t pwm);
int16_t motor_getpwm(uint8_t mot);
inline void motor_off(uint8_t mot) { motor_setpwm(mot, 0); }
void motor_allOff();

#endif /* MOTOR_H_ */

