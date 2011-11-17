#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

void motorcontrol_init();

float motorcontrol_getvel(int mot);
void motorcontrol_setvel(int mot, float rps);
void motorcontrol_setvel2(float vel0, float vel1);
void motorcontrol_stop(int motnum);
void motorcontrol_stop();
void motorcontrol_disable(int mot);
void motorcontrol_setDebug(bool new_debug);
float sign(float in);

#endif

