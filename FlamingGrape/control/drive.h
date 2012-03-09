#ifndef DRIVE_H_
#define DRIVE_H_

void drive(float leftvel, float rightvel);
void drive_dist(float leftvel, float rightvel, float ldist, float rdist, bool traj=true);
void drive_stop();
void drive_off();

void drive_fd(float vel);
void drive_fdDist(float vel, float dist, bool traj=true);
void drive_bk(float vel);
void drive_bkDist(float vel, float dist, bool traj=true);
void drive_lturn(float vel);
void drive_lturnDeg(float vel, float deg, bool traj=true);
void drive_rturn(float vel);
void drive_rturnDeg(float vel, float deg, bool traj=true);

void drive_steer(float steer, float vel);

void drive_waitDist(float dist);

void drive_setTrajAmax(float amax);
float drive_getTrajAmax();

#endif
