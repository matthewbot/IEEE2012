#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

#include <stdint.h>

void controlpanel_init();

void controlpanel();
void controlpanel_sensor();
void controlpanel_drive();
void controlpanel_algorithm();
void controlpanel_tests();

void controlpanel_pwmtest();
void controlpanel_pwmtest_single(int16_t pwm);

#endif /* CONTROLPANEL_H_ */
