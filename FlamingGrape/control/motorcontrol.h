#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "hw/motor.h"
#include "control/pid.h"

static const int motorcontrol_count = 2;

void motorcontrol_init();

float motorcontrol_getRPS(int motnum);
float motorcontrol_getRPSDesired(int motnum);
void motorcontrol_setRPS(int motnum, float RPS);
inline void motorcontrol_stop(int mot) { motorcontrol_setRPS(mot, 0); }

void motorcontrol_setEnabled(bool enabled=true);
void motorcontrol_setGains(const PIDGains &newpidgains);
PIDGains motorcontrol_getGains();
void motorcontrol_setDebug(bool debug);

void motorcontrol_tick();

#endif

