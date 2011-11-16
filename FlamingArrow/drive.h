#ifndef DRIVE_H_
#define DRIVE_H_

void drive_fwd(float dist, float vel);
void drive_bck(float dist, float vel);
void drive_l_turn(float degrees, float vel);
void drive_r_turn(float degrees, float vel);
void drive_l_piv_bck(float degrees, float vel);
void drive_l_piv_fwd(float degrees, float vel);
void drive_r_piv_bck(float degrees, float vel);
void drive_r_piv_fwd(float degrees, float vel);

#endif
