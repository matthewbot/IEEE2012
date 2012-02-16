#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "hw/motor.h"
#include "control/pid.h"

void motorcontrol_init();

float motorcontrol_getrps(int mot);
void motorcontrol_setrps(int mot, float rps);
inline void motorcontrol_stop(int mot) { motorcontrol_setrps(mot, 0); }

void motorcontrol_setEnabled(bool enabled=true);
void motorcontrol_setGains(const PIDGains &newpidgains);
PIDGains motorcontrol_getGains();
void motorcontrol_setDebug(bool debug);

void motorcontrol_tick();

#endif

