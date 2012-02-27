#ifndef DRIVE_H_
#define DRIVE_H_

void drive(float leftvel, float rightvel);
void drive_dist(float leftvel, float rightvel, float dist, int distm);
void drive_stop();
void drive_off();

void drive_fd(float vel);
void drive_fd_dist(float vel, float dist);
void drive_bk(float vel);
void drive_bk_dist(float vel, float dist);
void drive_lturn(float vel);
void drive_lturn_deg(float vel, float deg);
void drive_rturn(float vel);
void drive_rturn_deg(float vel, float deg);

void drive_steer(float steer, float vel);

void drive_wait_dist(float dist);

#endif
