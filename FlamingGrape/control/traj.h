#ifndef TRAJ_H
#define TRAJ_H

#include "hw/motor.h"

void traj_tick();

void traj_goDist(float ldist, float lfinalrps, float lvmax, float lamax, 
                 float rdist, float rfinalrps, float rvmax, float ramax);
void traj_goVel(float lfinalrps, float lamax,
                float rfinalrps, float ramax);
void traj_stop();
void traj_wait();

#endif
