#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

void motorcontrol_init();

float motorcontrol_getvel(int mot);
void motorcontrol_setvel(int mot, float rps);
void motorcontrol_disable(int mot);

#endif

