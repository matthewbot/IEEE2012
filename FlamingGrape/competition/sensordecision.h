#ifndef SENSORDECISION_H
#define SENSORDECISION_H

#include <stdint.h>

bool sensordecision_verifyOk(); // checks to make sure all four sensors are online and responding
void sensordecision_prepare(uint8_t decision); // decision is 0 to 3
bool sensordecision_available(); // true if decision is available
bool sensordecision_wait(); // waits for a decision to be available, returns false if not available within a second
bool sensordecision_isRight();

#endif
