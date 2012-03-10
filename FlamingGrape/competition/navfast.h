#ifndef NAVFAST_H
#define NAVFAST_H

void navfast_lap();

bool navfast_loopback();
bool navfast_leftright(bool right);
bool navfast_cross(bool right); // right means we're currently on the right, crossing to the left
bool navfast_jump(bool right);
void navfast_end(bool right);

#endif
