#ifndef TRAJ_H
#define TRAJ_H

#include "hw/motor.h"

void traj_tick();

void traj_setup(Motor mot, float dist, float final_rps, float vmax, float amax);
void traj_setEnabled(bool enabled);
void traj_wait();

#endif
