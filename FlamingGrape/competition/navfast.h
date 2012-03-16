#ifndef NAVFAST_H
#define NAVFAST_H

#include <stdint.h>

void navfast_lap();

bool navfast_loopback(uint8_t hlap);
bool navfast_leftright(bool right, uint8_t hlap);
bool navfast_cross(bool right); // right means we're currently on the right, crossing to the left
bool navfast_jump(bool right);
void navfast_end(bool right);

#endif
